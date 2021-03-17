#! /usr/bin/env python2
import math
import numpy as np
import rospy

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry


class map_generator(object):

    def __init__(self):
        """ Constructor """
        self.map = None
        self.points = []
        self.has_odom = False
        self.has_map = False
        self.init_params()
        self.floor_bias = 1.0

    def init_params(self):
        """ Initializes ros parameters """
        
        # rospack = rospkg.RosPack()
        # package_path = rospack.get_path("yolov4_trt_ros")
        self.dimx = rospy.get_param("~map/x_size", default=20.0)
        self.dimy = rospy.get_param("~map/y_size", default=20.0)
        self.dimz = rospy.get_param("~map/z_size", default=2.50)
        self.map_rand = rospy.get_param("~map/random", default=True)
        self.narrow_v = rospy.get_param("~narrow_gap_vertical", default=0.8)
        self.resolution = rospy.get_param("~resolution", default=0.05)
        self.narrow_h = rospy.get_param("~narrow_gap_horizontal", default=0.8)
        self.step_size = rospy.get_param("~step_size", default=1.8)
        self.end_blank = rospy.get_param("~map/end_blank", default=1.5)


        self.narrow_gap_width = rospy.get_param("~narrow_gap_width", default=3.0)
        self.gap_num = rospy.get_param("~gap_num", default=3)

        self.wall_num = rospy.get_param("~wall_num", default=3)
        self.wall_width = rospy.get_param("~wall_width", default=3.0)


        self.rand_obs_scale = rospy.get_param("~rand_obs_scale", default=3.0)
        self.rand_obs_num = rospy.get_param("~rand_obs_num", default=5)
        
        

        if (self.map_rand == False):
            self.random = np.random.RandomState(0)
        else:
            self.random = np.random

        print("narrow_v: ", self.narrow_v)

        self.init_x = rospy.get_param("init_state_x", default=0.0)
        self.init_y = rospy.get_param("init_state_y", default=0.0)

        


        self.add_floor = rospy.get_param("add_floor", default=True)
        self.add_ceiling = rospy.get_param("add_ceiling", default=True)
        
        self.all_map_topic = rospy.get_param("~all_map_topic", default="random_forest/all_map")
        self.all_map_pub = rospy.Publisher(self.all_map_topic, PointCloud2, queue_size=1)
        # self.odom_sub = rospy.Subscriber( "odometry", Odometry, self.odom_callback, queue_size=50);

        self.rate = rospy.get_param("sensing/rate", default=1.0);
        self.rate = rospy.Rate(self.rate)

        print("dimx: ", self.dimx)
        print("dimy: ", self.dimy)
        print("dimz: ", self.dimz)

    def add_box(self, size, position):
        ''' size: [x,y,z]
            position: [x,y,z] --- center position
        '''
        # print("=====================")
        # print("box size: ", size)
        # print("box position: ", position)
        # print("=====================")
        position[2] -= self.floor_bias
        x_low = math.floor((position[0] - size[0] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        x_high = math.floor((position[0] + size[0] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        y_low = math.floor((position[1] - size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        y_high = math.floor((position[1] + size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        z_low = math.floor((position[2] - size[2] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        z_high = math.floor((position[2] + size[2] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution

        x = x_low
        while x <= x_high:
            y = y_low
            while y <= y_high:
                z = z_low
                while z <= z_high:
                    if (math.fabs(x - x_low) < self.resolution) or (math.fabs(x - x_high) < self.resolution) \
                        or (math.fabs(y - y_low) < self.resolution) or (math.fabs(y - y_high) < self.resolution) \
                        or (math.fabs(z - z_low) < self.resolution) or (math.fabs(z - z_high) < self.resolution):
                        self.points.append([x,y,z])
                    z += self.resolution
                y += self.resolution
            x += self.resolution

        return

    def add_cylinder(self, size, position):
        ''' size: [r, h]
            position: [x,y,z] --- center position
        '''
        position[2] -= self.floor_bias
        center_x = position[0]
        center_y = position[1]
        z_low = math.floor((position[2] - size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        z_high = math.floor((position[2] + size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution

        radius_num = math.floor(size[0] / self.resolution)
        x = - radius_num
        while x <= radius_num:
            y = - radius_num
            while y <= radius_num:
                radius2 = x ** 2 + y ** 2
                if radius2 < (radius_num + 0.5) ** 2:
                    z = z_low
                    while z <= z_high:
                        if radius2 > (radius_num - 0.5) ** 2 or \
                            (math.fabs(z - z_low) < self.resolution) or (math.fabs(z - z_high) < self.resolution):
                            self.points.append([center_x + x * self.resolution, center_y + y * self.resolution, z])
                        z += self.resolution
                y += 1
            x += 1

        return

    def add_layer(self, size, position):
        ''' size: [x,y]
            position: [x,y,z] --- center position
        '''        
        x_low = math.floor((position[0] - size[0] / 2) / self.resolution) * self.resolution + 0.5 * self.resolution
        x_high = math.floor((position[0] + size[0] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        y_low = math.floor((position[1] - size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        y_high = math.floor((position[1] + size[1] / 2) / self.resolution) * self.resolution  + 0.5 * self.resolution
        z = position[2]

        x = x_low
        while x <= x_high:
            y = y_low
            while y <= y_high:
                self.points.append([x,y,z])
                y += self.resolution
            x += self.resolution
        
        return

    def add_chair(self, position, scale):
        '''position: [x, y] '''
        x = self.random.rand() * 0.2 + 0.4
        h = self.random.rand() * 0.1 + 0.4

        x = x * scale
        h = h * scale

        self.add_box_on_floor([x, x, h], [position[0], position[1], h/2])
        direction = self.random.randint(0, 5)
        h_ratio = 2 # 1.5 + np.random.rand() * 0.5
        if direction == 0:
            return
        elif direction == 1:
            self.add_box_on_floor([x, 0.1, h * h_ratio], [position[0], position[1] + x / 2, h * h_ratio / 2])
        elif direction == 2:
            self.add_box_on_floor([x, 0.1, h * h_ratio], [position[0], position[1] - x / 2, h * h_ratio / 2])
        elif direction == 3:
            self.add_box_on_floor([0.1, x, h * h_ratio], [position[0] + x / 2, position[1] , h * h_ratio / 2])
        elif direction == 4:
            self.add_box_on_floor([0.1, x, h * h_ratio], [position[0] - x / 2, position[1], h * h_ratio / 2])

    def add_bed(self, position):
        ''' position[x,y]'''
        x = np.random.rand() * 0.6 + 1.6
        y = np.random.rand() * 0.6 + 1.2
        z = np.random.rand() * 0.2 + 0.3
        # left right
        if np.random.rand() > 0.5:
            self.add_box_on_floor([x, y, z], [-self.dimx / 2 + x / 2, position[1], z/2])
            if np.random.rand() > 0.3:
                self.add_box_on_floor([0.1, y, z*1.5], [-self.dimx / 2 + 0.1, position[1], z*1.5/2])
        else:
            self.add_box_on_floor([x, y, z], [self.dimx / 2 - x / 2, position[1], z/2])
            if np.random.rand() > 0.3:
                self.add_box_on_floor([0.1, y, z*1.5], [self.dimx / 2 - 0.1, position[1], z*1.5/2])

    def add_table(self, position, scale):
        ''' position[x,y]'''
        x, y, z = self.random.rand(3)
        ratio = 0.3 + self.random.rand() * 0.5

        x = self.random.rand() * 1.0 + 0.4
        y = self.random.rand() * 1.0 + 0.4
        z = z * 0.6 + 0.4

        x = x * scale
        y = y * scale
        z = z * scale
        # upper partself.rand_obs_scale
        # lower part
        if np.random.rand() > 0.5:
            self.add_cylinder_on_floor([x/2 * ratio, z], [position[0] * 0.6, position[1]])

        else:
            self.add_box_on_floor([x * ratio, y * ratio, z], [position[0] * 0.6, position[1]])

    def add_long_door(self, position):
        ''' position[x,y]'''
        # warning: need to check map
        x = np.random.rand() * 0.4 + 0.8
        y = np.random.rand() * 0.4 + 0.4

        pos_x = (position[0] + x / 2) + np.random.rand() * (self.dimx - x)
        pos_y = position[1] + self.step_size / 2.0
        # left side
        self.add_box_on_floor([pos_x - x / 2 + self.dimx / 2, y, self.dimz], [-self.dimx / 2 + (pos_x - x / 2 + self.dimx / 2) / 2, pos_y, self.dimz / 2])
        # right side
        self.add_box_on_floor([self.dimx / 2 - pos_x - x / 2, y, self.dimz], [self.dimx / 2 - (self.dimx / 2 - pos_x - x / 2) / 2, pos_y, self.dimz / 2])


    def add_narrow_door(self, position):
        ''' position: y'''
        # warning: need to check map
        left_len = self.dimx * 1.0 / (6 * 2) + (self.dimx - self.narrow_gap_width - self.dimx * 1.0 / 6) * np.random.rand();
        right_len = self.dimx - self.narrow_gap_width - left_len;
    

        self.add_box([left_len, 2, self.dimz],[-self.dimx / 2 + 1 / 2.0 * left_len, position, self.dimz / 2.0])
        self.add_box([right_len, 2, self.dimz],[self.dimx / 2 - 1 / 2.0 * right_len, position, self.dimz / 2.0])

    def add_narrow_door_2(self, position):
        ''' position: y'''
        # warning: need to check map
        left_len = self.dimx * 1.0 / (6*3)  + (self.dimx - self.narrow_gap_width * 2 - self.dimx * 1.0 / 6) * np.random.rand()
        middle_len = self.dimx * 1.0 / (6*3)  + ((self.dimx - self.narrow_gap_width * 2 - self.dimx * 1.0 / 6) - (left_len - self.dimx * 1.0 / (6*3))) * np.random.rand()
        right_len = self.dimx - self.narrow_gap_width * 2 - left_len - middle_len;
    

        self.add_box([left_len, 2, self.dimz],[-self.dimx / 2 + 1 / 2.0 * left_len, position, self.dimz / 2.0])
        self.add_box([middle_len, 2, self.dimz],[-self.dimx/ 2 + left_len + self.narrow_gap_width + middle_len * 1 / 2.0, position, self.dimz / 2.0])
        self.add_box([right_len, 2, self.dimz],[self.dimx / 2 - 1 / 2.0 * right_len, position, self.dimz / 2.0])
    
    def add_narrow_door_n(self, position, wall_width, gap_width, gap_num):
        ''' position: y'''
        b_num  = gap_num + 1
        min_len = (self.dimx - gap_width * gap_num) / b_num * (1.0 / 6)
        allo_len = (self.dimx - gap_width * gap_num) * (5.0 / 6)
        r = self.random.dirichlet(np.ones(b_num),size=1)

        x_acc = .0
        for i in range(b_num):
            len = min_len + allo_len * r[0][i]
            size = [len, wall_width, self.dimz]
            x_pos = -self.dimx / 2 + x_acc + i * gap_width + 1 / 2.0 * len
            x_acc = x_acc + len;
          
            self.add_box(size, [x_pos, position, self.dimz / 2.0])


    def add_box_on_floor(self, size, position):
        ''' size: [x, y, z]
        position: [x, y] '''
        self.add_box(size, position + [size[-1] / 2])

    def add_random_box(self, position, scale):
        ''' position: [x, y] '''
        x = 10
        y = 10
        z = 10
        while x + y + z > 4:
            x = self.random.rand() * 0.7 + 0.3
            y = self.random.rand() * 0.7 + 0.4
            z = self.random.rand() * 1.4 + 0.4

        x = x * scale
        y = y * scale
        z = z * scale
        self.add_box_on_floor([x,y,z], position)

    def add_random_cylinder(self, position, scale):
        ''' position: [x, y] '''
        r = 10
        h = 10 
        while r * 2 + h > 3:
            r = self.random.rand() * 0.35 + 0.15
            h = self.random.rand() * 1.2 + 0.8

        r = r * scale
        h = h * scale

        self.add_cylinder_on_floor([r,h], position)

    def add_stride_on_ceiling(self, position):
        ''' 
        position: [x, y] '''
        y = np.random.rand() * 0.2 + 0.4
        z = np.random.rand() * 0.4 + 0.2
        self.add_box([self.dimx, y, z], [0, position[1], self.dimz - z / 2])

    def add_narrow_stride(self, position):
        ''' 
        position: [x, y] '''
        y =  np.random.rand() * 0.2 + 0.4
        z = np.random.rand() * (self.dimz - self.narrow_v)
        self.add_box([self.dimx, y, z], [0, position[1], self.dimz - z / 2])

        y = np.random.rand() * 0.2 + 0.4
        z = self.dimz - z - self.narrow_v;
        self.add_box([self.dimx, y, z], [0, position[1], 0 + z / 2])

    def add_stride_on_floor(self, position):
        '''
        position: [x, y] '''
        y = np.random.rand() * 0.2 + 0.4
        z = np.random.rand() * 0.4 + 0.2
        self.add_box([self.dimx, y, z], [0, position[1], 0 + z / 2])

    def add_cylinder_on_floor(self, size, position):
        ''' size: [r, h]
        position: [x, y] '''
        self.add_cylinder(size, [position[0], position[1], size[1] / 2])

    def publish_map(self):
        if not self.has_map:
            return False
        self.all_map_pub.publish(self.map)
        return True

    def odom_callback(self, odom):
        self.has_odom = True
        self.publish_map()

    # def make_random_corridor(self):
    #     self.dimx = 3.0
    #     self.dimy = 50.0
    #     self.dimz = 2.0
    #     self.step_size = 1.8
    #     steps = math.floor(self.dimy / self.step_size)
    #     for i in range(int(steps)):
    #         # clear start and end
    #         if i < 1 or i > steps - 2:
    #             continue
    #         obs_num = np.random.randint(2, 3)

    #         center_x = - self.dimx / 2
    #         center_y = - self.dimy / 2 + self.step_size * i

    #         if np.random.rand() < 0.4:
    #             # self.add_narrow_stride([center_x, center_y])
    #             self.add_stride_on_floor([center_x, center_y])
    #         else:
    #             # self.add_stride_on_ceiling([center_x, center_y])
    #             self.add_long_door([center_x, center_y])
            

    #         # continue

    #         for num in range(obs_num):
    #             pos_x = center_x + 0.2 + np.random.rand() * (self.dimx - 0.4)
    #             pos_y = center_y + 0.3 + np.random.rand() * (self.step_size - 0.6)
    #             object_type = np.random.randint(0, 8)

    #             if object_type < 3:
    #                 self.add_random_box([pos_x, pos_y])
    #             # elif object_type == 1:
    #             #     self.add_chair([pos_x, pos_y])
    #             # elif object_type == 2:
    #             #     self.add_bed([pos_x, pos_y])
    #             elif object_type < 4:
    #                 self.add_stride_on_ceiling([center_x, center_y])
    #             elif object_type < 5:
    #                 self.add_stride_on_floor([pos_x, pos_y])
    #             # elif object_type == 5:
    #             #     self.add_table([pos_x, pos_y])
    #             # elif object_type == 6:
    #             #     self.add_random_box([pos_x, pos_y])
    #             elif object_type < 7:
    #                 self.add_random_cylinder([pos_x, pos_y])
    #             else:
    #                 self.add_random_box([pos_x, pos_y])

    def make_custom_corridor(self):
        steps = self.wall_num + 1
        self.step_size = (self.dimy - self.end_blank * 2) / steps;
        for i in range(int(steps)):

            center_y = - self.dimy / 2 + self.step_size * (i + 1) + self.end_blank

            if i < steps - 1:
                self.add_narrow_door_n(center_y, self.wall_width,  self.narrow_gap_width, self.gap_num)

            for num in range(self.rand_obs_num):
                pos_x = (-self.dimx / 2.0 + 2.0 ) + (self.dimx - 4.0) * self.random.rand() 
                pos_y = center_y - 2.0 - (self.step_size - 4.0) * self.random.rand()
                object_type = self.random.randint(0, 8)
                

                if object_type < 3:
                    self.add_random_box([pos_x, pos_y], self.rand_obs_scale)
                # elif object_type == 1:
                #     self.add_chair([pos_x, pos_y])
                # elif object_type == 2:
                #     self.add_bed([pos_x, pos_y])
                elif object_type < 4:
                    # self.add_stride_on_ceiling([center_x, center_y])
                    # self.add_stride_on_floor([pos_x, pos_y])
                    self.add_table([pos_x, pos_y], self.rand_obs_scale)
                elif object_type < 5:
                    self.add_chair([pos_x, pos_y], self.rand_obs_scale)
                # elif object_type == 5:
                #     self.add_table([pos_x, pos_y])
                # elif object_type == 6:
                #     self.add_random_box([pos_x, pos_y])
                elif object_type < 7:
                    self.add_random_cylinder([pos_x, pos_y], self.rand_obs_scale)
                else:
                    self.add_random_box([pos_x, pos_y], self.rand_obs_scale)


    def make_map(self):
        rospy.loginfo("start making map")
        # # # scene 0 : square room
        # self.dimx = 20
        # self.dimy = 20
        # self.diz = 2.0
        # for x in range(-4, 5):
        #     for y in range(-4, 5):
        #         # if x == 0 and y == 0:
        #         #     continue
        #         self.add_box([0.8, 0.8, self.dimz], [x * 2, y * 2, self.dimz / 2])

        # # scene 0.1 : square room random
        # CHANGE!
        # self.dimx = 22
        # self.dimy = 22
        # self.diz = 2.0
        # for x in range(-4, 5):
        #     for y in range(-4, 5):
        #         for num in range(np.random.randint(1,3)):
        #             dx, dy = (np.random.rand(2) - 0.5) * 1.2
        #             sizex, sizey = (np.random.rand(2) - 0.5) * 1.2
        #             sizez = np.array([1.0, (0.6 + 2 * np.random.rand())]).min() * self.dimz
        #             dz = -1.5 * np.random.rand()
        #             # if x == 0 and y == 0:i
        #             #     continue
        #             if np.random.rand() < 0.1:
        #                 continue
        #             elif np.random.rand() < 0.7:
        #                 self.add_box_on_floor([0.8 + sizex, 0.8 + sizey, sizez], [x * 2 + dx, y * 2 + dy])
        #             else:
        #                 self.add_cylinder_on_floor([(0.8 + sizex) / 3, self.dimz], [x * 2 + dx, y * 2 + dy])

        # # scene 1 : corridor
        self.make_custom_corridor()

        # floor
        # self.add_layer([self.dimx, self.dimy], [0,0,-0.5])
        # # ceiling
        # self.add_layer([self.dimx, self.dimy], [0,0, self.dimz - self.floor_bias])
        # boundary
        # left
        self.add_box([0.10, self.dimy, self.dimz], [-self.dimx / 2, 0, self.dimz / 2.0])
        # right
        self.add_box([0.10, self.dimy, self.dimz], [self.dimx / 2, 0, self.dimz / 2.0])
        # top
        self.add_box([self.dimx, 0.10, self.dimz], [0, self.dimy / 2, self.dimz / 2.0])
        # # button
        self.add_box([self.dimx, 0.10, self.dimz], [0, -self.dimy / 2, self.dimz / 2.0])

        # # add items
        # # # sofa 1
        # self.add_box([2.5, 0.2, 1.1], [-4, -3.8, 0.65])
        # self.add_box([2.5, 0.5, 0.5], [-4, -3.35, 0.3])

        # # table 1
        # self.add_box([1.5, 0.8, 0.6], [-4, -2.30, 0.3])

        # # table 2
        # self.add_cylinder([0.8, 0.6], [-3, -0.2, 0.4])

        # # chair 1
        # self.add_box([0.6, 0.6, 0.5], [-4.3, -0.5, 0.3])
        # self.add_box([0.1, 0.6, 1.4], [-4.55, -0.5, 0.75])

        # self.add_box([0.6, 0.6, 0.5], [-1.7, -0.5, 0.3])
        # self.add_box([0.1, 0.6, 1.4], [-1.45, -0.5, 0.75])

        # # washing machine
        # self.add_box([0.6, 0.9, 1.0], [-4.2, 2.3, 0.5])


        # # bed 1
        # self.add_box([2.5, 2.0, 0.5], [4.65, -2.0, 0.25])
        # # nightstand 1
        # self.add_box([0.5, 0.6, 0.6], [5.65, -0.3, 0.3])

        # # wall 0
        # self.add_box([0.8, 3.0, 1.2], [2.0, 2.4, 0.6])
        # # wall 1 / 2 / 3 (door)
        # self.add_box([3.5, 0.3, 2.0], [-4.3, 1.8, 1.0])
        # self.add_box([1.2, 0.3, 2.0], [-0.45, 1.8, 1.0])
        # self.add_box([0.3, 2.2, 2.0], [0, 2.9, 1.0])

        # #simple scene 1 : krrt fails
        # self.add_box([0.5, 8, self.dimz], [1.25, -1.25, self.dimz / 2])
        # self.add_box([10, 0.4, self.dimz], [0, -0.9, self.dimz / 2])
        # self.add_box([2, 2.5, 2], [4, 2.5, 1])




        # transfer to pcl
        self.map = PointCloud2()
        self.map.height = 1
        self.map.width = len(self.points)
        self.map.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        self.map.point_step = 12 #12
        self.map.row_step = 12 * len(self.points)
        self.map.is_bigendian = False
        self.map.is_dense = False
        self.map.data = np.asarray(self.points, np.float32).tostring()
        self.map.header.frame_id = "map"

        self.has_map = True
        rospy.loginfo("finish making map")

        return True
    
def main():
    
    rospy.init_node('random_map_sensing', anonymous=True)

    map_maker = map_generator()
    map_maker.init_params()
    map_maker.make_map()
    while not rospy.is_shutdown():
        map_maker.publish_map()
        map_maker.rate.sleep()

if __name__ == '__main__':
    main()

