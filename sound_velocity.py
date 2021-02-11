from auvlib.data_tools import std_data, gsf_data, xtf_data, all_data, xyz_data, csv_data, benchmark
from auvlib.bathy_maps import mesh_map , base_draper #, draw_map # note that draw_map cannot include together with benchmark. Hence choose to comment out one while calling different functions.
import sys
import matplotlib.pyplot as plt
import numpy as np
import argparse
import copy

# read sound speed in vehicleCTD.txt file
def plot_raw_vehicleCTD_sound_speed():
    with open(sys.argv[1],"r") as f:
        data = zip(*[line.split() for line in f])
        depth = [float(x) for x in data[3]]           # 4th column as depth
        sound_speed = [float(x) for x in data[4]]     # 5th column as sound speed
    
    # plot 
    """
    plt.figure(dpi=300, figsize=(10, 6))
    plt.plot(depth, sound_speed, 'bo', markersize=0.1)
    # plt.tick_params(axis='both',labelsize=500)
    plt.xlabel('depth')
    plt.ylabel('velocity')
    plt.title('data in vehicleCTD.txt')
    plt.savefig("vehicleCTD_sound_velocity.png")
    """

    # fit y = ax + b in the depth range [55, 75]
    """
    xy  = zip(depth, sound_speed)
    xy_filtered = zip(*[i for i in xy if (i[0] > 55. and i[0] <75.)])
    ab = np.polyfit(xy_filtered[0], xy_filtered[1], 1)

    print(ab) # -> [1.29864668e-02 1.47689697e+03]   
    """

# read sound speed in XYZ88 datagram
def plot_soundSpeed_vs_depth():
    all_pings = all_data.all_mbes_ping.parse_file(sys.argv[1])

    # for each ping, find the z when y = 0
    # find the y1 < 0 < y2, and interpolate the z
    depth_vehicle_to_seafloor = []
    for ping in all_pings:
        point1 = min([point for point in ping.beams if point[1] >= 0], key=lambda x:x[1])
        point2 = max([point for point in ping.beams if point[1] < 0], key=lambda x:x[1])
        depth_vehicle_to_seafloor.append(abs(np.interp(0, [point1[1], point2[1]], [point1[2],point2[2]])))


    velocity = [x.sound_vel_ for x in all_pings]
    transducer_depth = [x.transducer_depth_ for x in all_pings]
    print(min(transducer_depth), max(transducer_depth))
    total_depth = [x + y for (x, y) in zip(depth_vehicle_to_seafloor, transducer_depth)] 

    plt.figure()

    plt.subplot(321)
    plt.plot(velocity)
    plt.xlabel('no. ping')
    plt.ylabel('velocity')

    plt.subplot(323)
    plt.plot(transducer_depth)
    plt.xlabel('no. ping')
    plt.ylabel('transducer depth')

    plt.subplot(324)
    plt.plot(total_depth)
    plt.xlabel('no. ping')
    plt.ylabel('sea total depth')

    plt.subplot(325)
    plt.plot(transducer_depth, velocity, 'bo', markersize=0.22)
    plt.xlabel('transducer depth')
    plt.ylabel('velocity')

    plt.subplot(326)
    plt.plot(total_depth, velocity, 'bo', markersize=0.22)
    plt.xlabel('sea total depth')
    plt.ylabel('velocity')
    plt.tight_layout(pad=0.4, w_pad=5.0, h_pad=2.0)
    plt.savefig("test14_new.png", dpi = 300)

# read sound speed in sound velocity profile datagram
def plot_sound_velocity_profile(file_str):
    sound_velocity_profile = all_data.all_sound_speed_profile.parse_file(file_str)
    depth = sound_velocity_profile[0].depth_
    sound_speed = sound_velocity_profile[0].sound_speed_

    plt.figure(dpi=300, figsize=(10, 6))
    plt.plot(depth, sound_speed, 'bo', markersize=1, label='sound velocity profile')
    # plt.tick_params(axis='both',labelsize=500)
    plt.xlim(0, 130000)
    plt.ylim(14300, 14800)
    plt.xlabel('depth')
    plt.ylabel('velocity')
    plt.title('data saved in sound velocity profile')

    plt.axhline(y =14777, label = '(almost) constant in the multibeam velocity', linewidth=0.5)
    plt.legend()
    plt.savefig("sv profile in " + file_str[:-4])

# read sound speed in SVP_all.svp
def plot_svp_all_svp_file(args):
    sound_speeds = csv_data.csv_asvp_sound_speed.parse_file(args.svp_all_svp_file)

    # plot
    """
    plt.figure(dpi=300, figsize=(10, 6))
    plt.plot(sound_speeds[0].dbars, sound_speeds[0].vels, 'bo', markersize=0.5)
    plt.xlabel('depth')
    plt.ylabel('velocity')
    plt.title('data in SVP_all.svp')
    # plt.tick_params(axis='both',labelsize=500)
    plt.savefig("SVP_all_dotSvp_sound_velocity.png")
    """

    # fit y = ax + b in the depth range [55, 75]
    """"""
    xy  = zip(sound_speeds[0].dbars, sound_speeds[0].vels)
    xy_filtered = zip(*[i for i in xy if (i[0] > 55. and i[0] <75.)])
    ab = np.polyfit(xy_filtered[0], xy_filtered[1], 1)

    print(ab) # -> [1.71773480e-02 1.48058942e+03] 
    

def modify_sound_velocity(all_pings_original, id):
    # overwrite all the speed accoring to the ab, sound speed = ab[0] * depth + ab[1]
    # overwrite point (x, y, z) for each beams since the point is relative distance linear with sound velocity
    S2X=3.520
    S2Y=-0.004
    S2Z=0.330
    offset_x = 0.011 + S2X # positive direction is forward
    offset_y = -(0.000 + S2Y) # positive direction is left
    offset_z = -(-0.006 + S2Z) # positive direction is up, so there RX is mounted above the RX array
    offset = [offset_x, offset_y, offset_z]

    all_pings = all_pings_original[:]
    # print("method " + str(id) + ": (before)", all_pings[0].beams[0])
    for i in range(len(all_pings)):
        original_sound_vel = all_pings[i].sound_vel_

        if id == 0:
            continue
        else:
            all_pings[i].sound_vel_ = 10000 + id * 100.
        
        ''' 
        # zoom in [14300, 15500]
        elif id == 1:
           # method 1: linear fit to vehicleCTD.txt
           ab = [1.29864668e-02, 1.47689697e+03]
           all_pings[i].sound_vel_ = (ab[0] * all_pings[i].transducer_depth_ + ab[1]) * 10. # *10 to convert to the same unit
        elif id == 2:
            # method 2: constant to around 14817
            all_pings[i].sound_vel_ = 14300.
        elif id == 3:
            # method 2: constant to around 14817
            all_pings[i].sound_vel_ = 14400.
        elif id == 4:
            # method 2: constant to around 14817
            all_pings[i].sound_vel_ = 14500.
        elif id == 5:
            # method 2: constant to around 14817
            all_pings[i].sound_vel_ = 14600.
        elif id == 6:
            # method 3: constant to around ~
            all_pings[i].sound_vel_ = 14700.
        elif id == 7:
            # method 4: constant to around ~
            all_pings[i].sound_vel_ = 14800.
        elif id == 8:
            # method 5: constant to around ~
            all_pings[i].sound_vel_ = 14900.
        elif id == 9:
            # method 6: constant to around ~
            all_pings[i].sound_vel_ =  15000.
        elif id == 10:
            # method 7: constant to around ~
            all_pings[i].sound_vel_ = 15100.
        elif id == 11:
            # method 7: constant to around ~
            all_pings[i].sound_vel_ = 15200.
        elif id == 12:
            # method 7: constant to around ~
            all_pings[i].sound_vel_ = 15300.
        elif id == 13:
            # method 7: constant to around ~
            all_pings[i].sound_vel_ = 15400.
        elif id == 14:
            # method 7: constant to around ~
            all_pings[i].sound_vel_ = 15500.
        elif id == 15:
            # method 8: linear fit to SVP_all.svp
            ab = [1.71773480e-02, 1.48058942e+03]
            all_pings[i].sound_vel_ = (ab[0] * all_pings[i].transducer_depth_ + ab[1]) * 10. # *10 to convert to the same unit        
        else:
            print("wrong method id!")
        '''

        ratio = all_pings[i].sound_vel_ / original_sound_vel
        all_pings[i].beams = [ratio*(beam - offset) + offset for beam in all_pings[i].beams]
    
    # print("method " + str(id) + ": (after)", all_pings[0].beams[0])
    return all_pings

def plot_mesh_from_dotall_file(args):
    all_folder = args.folder
    all_pings_origin = all_data.all_mbes_ping.parse_folder(all_folder)
    all_data.write_data(all_pings_origin, "merged.cereal")
    all_entries = all_data.all_nav_entry.parse_folder(all_folder)
    bm = benchmark.track_error_benchmark()

    for i in range(100):
        all_pings = modify_sound_velocity(all_data.all_mbes_ping.read_data("merged.cereal"), i)
        mbes_pings = all_data.convert_matched_entries(all_pings, all_entries)
        if i == 0:
            bm.add_ground_truth(mbes_pings)
        else:
            bm.add_benchmark(mbes_pings, "method "+str(i).zfill(3))
        # if i == 15:
            # V, F, bounds = mesh_map.mesh_from_pings(mbes_pings, .5) # .5 is resolution of mesh in meters
            # mesh_map.show_mesh(V, F)
    bm.print_summary()
    
def plot_raw_range_and_beam_angle(file_str):
    raw_range_and_beam_angle = all_data.all_raw_range_and_beam_angle.parse_file(file_str)
    # figure 1: sound velocity
    velocity = [x.sound_vel_ for x in raw_range_and_beam_angle]
    plt.figure(dpi=300, figsize=(10, 6))
    plt.plot(velocity, 'bo', markersize=1)
    plt.xlabel('ping idx')
    plt.ylabel('sound velocity at transducer ')
    plt.title('data saved in raw range and beam angle')
    plt.savefig("raw range and beam angle - sv with transmit " + file_str[:-4])
    
    # figure 2: two way travel time of all beams in the first ping
    received_beam_ = raw_range_and_beam_angle[0].received_beam_
    beam_pointing_angle = [x[0] for x in received_beam_]
    two_way_tranvel_time_ = [x[5] for x in received_beam_]

    plt.figure(dpi=300, figsize=(10, 6))
    plt.subplot(211)
    plt.plot(two_way_tranvel_time_, 'bo', markersize=1)
    plt.xlabel('beam idx')
    plt.ylabel('two way travel time in s')
    plt.subplot(212)
    plt.plot(beam_pointing_angle, two_way_tranvel_time_, 'bo', markersize=1)
    plt.xlabel('beam pointing angle in 0.01 degree unit')
    plt.ylabel('two way travel time in s')
    plt.title('data saved in raw range and beam angle')
    plt.savefig("raw range and beam angle[0] - two way travel time with transmit " + file_str[:-4])

# the input is original xyz88, raw range and beam angle, attitude and nav position data
def plot_raw_mesh_file(all_mbes_ping, raw_range_and_beam_angle, all_nav_attitude, all_nav_entry):
    # convert raw_range_and_beam_angle to raw_all_mbes_ping with roll/pitch corrected by info from all_nav_attitude
    raw_all_mbes_ping = all_data.raw_range_and_beam_angle_convert_to_pings(raw_range_and_beam_angle, all_nav_attitude)
    # add trasducer depth and heading info from all_nav_entry to it
    raw_all_mbes_ping = all_data.raw_pings_add_transducer_depth_and_heading_to_pings(raw_all_mbes_ping, all_nav_entry)
    
    print('---------------------------------------------------------xyz data')
    xyz_data_mbes_pings= all_data.convert_matched_entries(all_mbes_ping, all_nav_entry)
    V_xyz, F_xyz, bounds_xyz = mesh_map.mesh_from_pings(xyz_data_mbes_pings, .5) # .5 is resolution of mesh in meters
    mesh_map.show_mesh(V_xyz, F_xyz)
    d2 = draw_map.BathyMapImage(xyz_data_mbes_pings, 500, 500) # create a bathymetry height map
    d2.draw_height_map(xyz_data_mbes_pings) # draw the height map
    d2.draw_track(xyz_data_mbes_pings) # draw the track of the vehicle
    d2.write_image("3_height_map_from_xyz88.png") # save the height map to "height_map.png"

    print('---------------------------------------------------------raw data')
    mbes_pings = all_data.raw_convert_matched_entries(raw_all_mbes_ping, all_nav_entry)
    V, F, bounds = mesh_map.mesh_from_pings(mbes_pings, .5) # .5 is resolution of mesh in meters
    mesh_map.show_mesh(V, F)
    d = draw_map.BathyMapImage(mbes_pings, 500, 500) # create a bathymetry height map
    d.draw_height_map(mbes_pings) # draw the height map
    d.draw_track(mbes_pings) # draw the track of the vehicle
    d.write_image("3_height_map_from_raw_data.png") # save the height map to "height_map.png"

    print('---------------------------------------------------------raw data with original_convert_function')
    mbes_pings_rotate = all_data.raw_pings_rotate_to_xyz88_direction(raw_all_mbes_ping)
    mbes_pings_orig_convert = all_data.convert_matched_entries(mbes_pings_rotate, all_nav_entry)
    V_orig_convert, F_orig_convert, bounds_orig_convert = mesh_map.mesh_from_pings(mbes_pings_orig_convert, .5) # .5 is resolution of mesh in meters
    mesh_map.show_mesh(V_orig_convert, F_orig_convert)
    d1 = draw_map.BathyMapImage(mbes_pings_orig_convert, 500, 500) # create a bathymetry height map
    d1.draw_height_map(mbes_pings_orig_convert) # draw the height map
    d1.draw_track(mbes_pings_orig_convert) # draw the track of the vehicle
    d1.write_image("3_height_map_from_raw_data_original_convert_function.png") # save the height map to "height_map.png"

def plot_raw_mesh_file_of_one_file(file_str):
    # read all_mbes_ping (for reference), raw_range_and_beam_angle, all_nav_attitude, all_nav_entry
    all_mbes_ping = all_data.all_mbes_ping.parse_file(file_str)
    raw_range_and_beam_angle = all_data.all_raw_range_and_beam_angle.parse_file(file_str)
    all_nav_attitude = all_data.all_nav_attitude.parse_file(file_str)
    all_nav_entry = all_data.all_nav_entry.parse_file(file_str)
    plot_raw_mesh_file(all_mbes_ping, raw_range_and_beam_angle, all_nav_attitude, all_nav_entry)

def plot_raw_mesh_file_of_all_files_in_the_folder(all_folder):
    all_mbes_ping = all_data.all_mbes_ping.parse_folder(all_folder)
    raw_range_and_beam_angle = all_data.all_raw_range_and_beam_angle.parse_folder(all_folder)
    all_nav_attitude = all_data.all_nav_attitude.parse_folder(all_folder)
    all_nav_entry = all_data.all_nav_entry.parse_folder(all_folder)
    plot_raw_mesh_file(all_mbes_ping, raw_range_and_beam_angle, all_nav_attitude, all_nav_entry)



if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--folder', action="store", dest="folder", type=str, default="/home/carol/Downloads/Mission_58_20190618_6/all_files/",
                        help='can read files from the folder')
    parser.add_argument('--svp_all_svp_file', action="store", dest="svp_all_svp_file", type=str, default="xxx/SVP_all.svp",
                        help='can read file SVP_all.svp')
    
    
    args = parser.parse_args()

    # plot_raw_vehicleCTD_sound_speed()
    # plot_soundSpeed_vs_depth()
    # plot_sound_velocity_profile(file_str)
    # plot_svp_all_svp_file(args)
    plot_mesh_from_dotall_file(args)
