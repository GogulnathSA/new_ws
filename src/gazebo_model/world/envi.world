<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <model name='cafe.world'>
      <pose>-0.490708 0.112147 0 0 -0 0</pose>
      <link name='Wall_10'>
        <collision name='Wall_10_Collision'>
          <geometry>
            <box>
              <size>2.75 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_10_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.75 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>4.875 -6.425 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_12'>
        <collision name='Wall_12_Collision'>
          <geometry>
            <box>
              <size>1.5 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_12_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>6.85 -6.425 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_13'>
        <collision name='Wall_13_Collision'>
          <geometry>
            <box>
              <size>3 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_13_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>3 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>7.525 -5 0 0 -0 1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_16'>
        <collision name='Wall_16_Collision'>
          <geometry>
            <box>
              <size>1.75 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_16_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.75 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>7.525 -2.775 0 0 -0 1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_18'>
        <collision name='Wall_18_Collision'>
          <geometry>
            <box>
              <size>2.75 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_18_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.75 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>6.225 -1.975 0 0 -0 3.14159</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_20'>
        <collision name='Wall_20_Collision'>
          <geometry>
            <box>
              <size>2.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_20_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.25 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>4.925 -0.925 0 0 -0 1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_22'>
        <collision name='Wall_22_Collision'>
          <geometry>
            <box>
              <size>2 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_22_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>4.925 1.05 0 0 -0 1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_24'>
        <collision name='Wall_24_Collision'>
          <geometry>
            <box>
              <size>3 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_24_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>3 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>6.35 1.975 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_25'>
        <collision name='Wall_25_Collision'>
          <geometry>
            <box>
              <size>2.75 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_25_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.75 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>7.775 3.275 0 0 -0 1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_27'>
        <collision name='Wall_27_Collision'>
          <geometry>
            <box>
              <size>2 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_27_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>7.775 5.5 0 0 -0 1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_28'>
        <pose>3.1 6.425 0 0 -0 3.14159</pose>
        <visual name='Wall_28_Visual_0'>
          <pose>-2.97975 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>3.5405 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_28_Collision_0'>
          <geometry>
            <box>
              <size>3.5405 0.15 2.5</size>
            </box>
          </geometry>
          <pose>-2.97975 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_28_Visual_1'>
          <pose>1.77025 0 0.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>5.9595 0.15 0.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_28_Collision_1'>
          <geometry>
            <box>
              <size>5.9595 0.15 0.5</size>
            </box>
          </geometry>
          <pose>1.77025 0 0.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_28_Visual_2'>
          <pose>2.17025 0 1.5 0 -0 0</pose>
          <geometry>
            <box>
              <size>5.1595 0.15 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_28_Collision_2'>
          <geometry>
            <box>
              <size>5.1595 0.15 2</size>
            </box>
          </geometry>
          <pose>2.17025 0 1.5 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_28_Visual_3'>
          <pose>-0.8095 0 1.9 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.8 0.15 1.2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_28_Collision_3'>
          <geometry>
            <box>
              <size>0.8 0.15 1.2</size>
            </box>
          </geometry>
          <pose>-0.8095 0 1.9 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_30'>
        <collision name='Wall_30_Collision'>
          <geometry>
            <box>
              <size>5.5 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_30_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>5.5 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-4.25 6.425 0 0 -0 3.14159</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_32'>
        <collision name='Wall_32_Collision'>
          <geometry>
            <box>
              <size>1 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_32_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-7.35 6.425 0 0 -0 3.14159</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_34'>
        <collision name='Wall_34_Collision'>
          <geometry>
            <box>
              <size>1.5 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_34_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-7.775 5.75 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_36'>
        <pose>-7.775 3.65 0 0 -0 -1.5708</pose>
        <visual name='Wall_36_Visual_0'>
          <pose>-0.99925 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.0015 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_36_Collision_0'>
          <geometry>
            <box>
              <size>1.0015 0.15 2.5</size>
            </box>
          </geometry>
          <pose>-0.99925 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_36_Visual_1'>
          <pose>0.50075 0 0.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.9985 0.15 0.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_36_Collision_1'>
          <geometry>
            <box>
              <size>1.9985 0.15 0.5</size>
            </box>
          </geometry>
          <pose>0.50075 0 0.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_36_Visual_2'>
          <pose>0.90075 0 1.5 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.1985 0.15 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_36_Collision_2'>
          <geometry>
            <box>
              <size>1.1985 0.15 2</size>
            </box>
          </geometry>
          <pose>0.90075 0 1.5 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_36_Visual_3'>
          <pose>-0.0985 0 1.9 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.8 0.15 1.2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_36_Collision_3'>
          <geometry>
            <box>
              <size>0.8 0.15 1.2</size>
            </box>
          </geometry>
          <pose>-0.0985 0 1.9 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_38'>
        <collision name='Wall_38_Collision'>
          <geometry>
            <box>
              <size>3.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_38_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>3.25 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-7.775 0.675 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_40'>
        <pose>-7.775 -2.3 0 0 -0 -1.5708</pose>
        <visual name='Wall_40_Visual_0'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>3 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_40_Collision_0'>
          <geometry>
            <box>
              <size>3 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_40_Visual_1'>
          <pose>0.37075 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>2.2585 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_40_Collision_1'>
          <geometry>
            <box>
              <size>2.2585 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0.37075 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_45'>
        <collision name='Wall_45_Collision'>
          <geometry>
            <box>
              <size>1.75 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_45_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.75 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-7.775 -4.525 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_47'>
        <pose>-3.765 -5.325 0 0 -0 3.14159</pose>
        <visual name='Wall_47_Visual_0'>
          <pose>-2.47475 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>3.2205 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_47_Collision_0'>
          <geometry>
            <box>
              <size>3.2205 0.15 2.5</size>
            </box>
          </geometry>
          <pose>-2.47475 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_47_Visual_1'>
          <pose>1.61025 0 0.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>4.9495 0.15 0.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_47_Collision_1'>
          <geometry>
            <box>
              <size>4.9495 0.15 0.5</size>
            </box>
          </geometry>
          <pose>1.61025 0 0.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_47_Visual_2'>
          <pose>2.01025 0 1.5 0 -0 0</pose>
          <geometry>
            <box>
              <size>4.1495 0.15 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_47_Collision_2'>
          <geometry>
            <box>
              <size>4.1495 0.15 2</size>
            </box>
          </geometry>
          <pose>2.01025 0 1.5 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_47_Visual_3'>
          <pose>-0.4645 0 1.9 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.8 0.15 1.2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <collision name='Wall_47_Collision_3'>
          <geometry>
            <box>
              <size>0.8 0.15 1.2</size>
            </box>
          </geometry>
          <pose>-0.4645 0 1.9 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_49'>
        <collision name='Wall_49_Collision'>
          <geometry>
            <box>
              <size>4.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_49_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>4.25 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-1.685 -1.435 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_50'>
        <collision name='Wall_50_Collision'>
          <geometry>
            <box>
              <size>3.88301 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_50_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>3.88301 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>0.43 -3.3 0 0 -0 -1.5306</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_52'>
        <collision name='Wall_52_Collision'>
          <geometry>
            <box>
              <size>3.5 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_52_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>3.5 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-2.15 5.075 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_53'>
        <collision name='Wall_53_Collision'>
          <geometry>
            <box>
              <size>1.75 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_53_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.75 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-0.475 4.275 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_55'>
        <collision name='Wall_55_Collision'>
          <geometry>
            <box>
              <size>1.74013 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_55_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.74013 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-0.475 2.68 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_60'>
        <collision name='Wall_60_Collision'>
          <geometry>
            <box>
              <size>0.15 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_60_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.15 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-0.475 1.885 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_62'>
        <collision name='Wall_62_Collision'>
          <geometry>
            <box>
              <size>5.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_62_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>5.25 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Bricks</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>-3.035 1.885 0 0 -0 3.14159</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_8'>
        <collision name='Wall_8_Collision'>
          <geometry>
            <box>
              <size>3.48 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_8_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>3.48 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>1.91 -5.325 0 0 -0 0</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <link name='Wall_9'>
        <collision name='Wall_9_Collision'>
          <geometry>
            <box>
              <size>1.25 0.15 2.5</size>
            </box>
          </geometry>
          <pose>0 0 1.25 0 -0 0</pose>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='Wall_9_Visual'>
          <pose>0 0 1.25 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.25 0.15 2.5</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
            <ambient>1 1 1 1</ambient>
          </material>
          <meta>
            <layer>0</layer>
          </meta>
        </visual>
        <pose>3.575 -5.875 0 0 -0 -1.5708</pose>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <static>1</static>
    </model>
    <model name='bookshelf'>
      <static>1</static>
      <link name='link'>
        <inertial>
          <mass>1</mass>
        </inertial>
        <collision name='back'>
          <pose>0 0.005 0.6 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.9 0.01 1.2</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual1'>
          <pose>0 0.005 0.6 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.9 0.01 1.2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='left_side'>
          <pose>0.45 -0.195 0.6 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.02 0.4 1.2</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual2'>
          <pose>0.45 -0.195 0.6 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.02 0.4 1.2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='right_side'>
          <pose>-0.45 -0.195 0.6 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.02 0.4 1.2</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual3'>
          <pose>-0.45 -0.195 0.6 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.02 0.4 1.2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='bottom'>
          <pose>0 -0.195 0.03 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.88 0.4 0.06</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual4'>
          <pose>0 -0.195 0.03 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.88 0.4 0.06</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='top'>
          <pose>0 -0.195 1.19 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.88 0.4 0.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual5'>
          <pose>0 -0.195 1.19 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.88 0.4 0.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='low_shelf'>
          <pose>0 -0.195 0.43 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.88 0.4 0.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual6'>
          <pose>0 -0.195 0.43 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.88 0.4 0.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='high_shelf'>
          <pose>0 -0.195 0.8 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.88 0.4 0.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual7'>
          <pose>0 -0.195 0.8 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.88 0.4 0.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>-2.62891 5.09124 0 0 -0 0</pose>
    </model>
    <model name='cafe_table'>
      <static>1</static>
      <link name='link'>
        <collision name='surface'>
          <pose>0 0 0.755 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.913 0.913 0.04</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <collision name='column'>
          <pose>0 0 0.37 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.042 0.042 0.74</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <collision name='base'>
          <pose>0 0 0.02 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.56 0.56 0.04</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://cafe_table/meshes/cafe_table.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>-4.45682 3.64828 0 0 -0 0</pose>
    </model>
    <model name='cafe_table_0'>
      <static>1</static>
      <link name='link'>
        <collision name='surface'>
          <pose>0 0 0.755 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.913 0.913 0.04</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <collision name='column'>
          <pose>0 0 0.37 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.042 0.042 0.74</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <collision name='base'>
          <pose>0 0 0.02 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.56 0.56 0.04</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://cafe_table/meshes/cafe_table.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>4.31475 4.45493 0 0 -0 0</pose>
    </model>
    <model name='cabinet'>
      <static>1</static>
      <link name='cabinet_bottom_plate'>
        <inertial>
          <pose>0 0 -1 0 -0 0</pose>
          <inertia>
            <ixx>2.05</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.05</iyy>
            <iyz>0</iyz>
            <izz>2.05</izz>
          </inertia>
          <mass>25</mass>
        </inertial>
        <collision name='cabinet_bottom_plate_geom'>
          <pose>0 0 0.01 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_visual'>
          <pose>0 0 0.01 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='cabinet_bottom_plate_geom_cabinet_back_plate'>
          <pose>0.235 0 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.02 0.45 1.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_cabinet_back_plate_visual'>
          <pose>0.235 0 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.02 0.45 1.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='cabinet_bottom_plate_geom_cabinet_left_plate'>
          <pose>0 0.235 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.02 1.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_cabinet_left_plate_visual'>
          <pose>0 0.235 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.02 1.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='cabinet_bottom_plate_geom_cabinet_middle_plate'>
          <pose>0 0 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_cabinet_middle_plate_visual'>
          <pose>0 0 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='cabinet_bottom_plate_geom_cabinet_right_plate'>
          <pose>0 -0.235 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.02 1.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_cabinet_right_plate_visual'>
          <pose>0 -0.235 0.51 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.02 1.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='cabinet_bottom_plate_geom_cabinet_top_plate'>
          <pose>0 0 1.01 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='cabinet_bottom_plate_geom_cabinet_top_plate_visual'>
          <pose>0 0 1.01 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.45 0.45 0.02</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>-1.23232 -1.68825 0 0 -0 0</pose>
    </model>
    <model name='fountain'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://fountain/meshes/fountain.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://fountain/meshes/fountain.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>2.74688 0.353258 0 0 -0 0</pose>
    </model>
    <model name='first_2015_trash_can'>
      <link name='link'>
        <inertial>
          <pose>0 0 0.3683 0 -0 0</pose>
          <mass>4.83076</mass>
          <inertia>
            <ixx>0.281534</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.281534</iyy>
            <iyz>0</iyz>
            <izz>0.126223</izz>
          </inertia>
        </inertial>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://first_2015_trash_can/meshes/trash_can.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://first_2015_trash_can/meshes/trash_can.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>-6.74765 -4.90373 0 0 -0 0</pose>
    </model>
    <model name='Mailbox'>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <mesh>
              <uri>model://mailbox/meshes/mailbox.dae</uri>
            </mesh>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://mailbox/meshes/mailbox.dae</uri>
            </mesh>
          </geometry>
          <material>
            <script>
              <uri>model://mailbox/materials/scripts</uri>
              <uri>model://mailbox/materials/textures</uri>
              <name>Mailbox/Diffuse</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <inertial>
          <pose>0 0 0 0 -0 0</pose>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
          <mass>1</mass>
        </inertial>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>-7.99778 -2.23277 0 0 -0 0</pose>
    </model>
    <model name='table'>
      <static>1</static>
      <link name='link'>
        <collision name='surface'>
          <pose>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual1'>
          <pose>0 0 1 0 -0 0</pose>
          <geometry>
            <box>
              <size>1.5 0.8 0.03</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Wood</name>
            </script>
          </material>
        </visual>
        <collision name='front_left_leg'>
          <pose>0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_left_leg'>
          <pose>0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='front_right_leg'>
          <pose>0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='front_right_leg'>
          <pose>0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_right_leg'>
          <pose>-0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_right_leg'>
          <pose>-0.68 -0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <collision name='back_left_leg'>
          <pose>-0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='back_left_leg'>
          <pose>-0.68 0.38 0.5 0 -0 0</pose>
          <geometry>
            <cylinder>
              <radius>0.02</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>4.66728 -4.06633 0 0 -0 0</pose>
    </model>
    <model name='cafe_table_1'>
      <static>1</static>
      <link name='link'>
        <collision name='surface'>
          <pose>0 0 0.755 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.913 0.913 0.04</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <collision name='column'>
          <pose>0 0 0.37 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.042 0.042 0.74</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <collision name='base'>
          <pose>0 0 0.02 0 -0 0</pose>
          <geometry>
            <box>
              <size>0.56 0.56 0.04</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <mesh>
              <uri>model://cafe_table/meshes/cafe_table.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
      <pose>-2.33796 -3.31048 0 0 -0 0</pose>
    </model>
    <state world_name='default'>
      <sim_time>257 439000000</sim_time>
      <real_time>262 465129454</real_time>
      <wall_time>1720411461 383415696</wall_time>
      <iterations>257439</iterations>
      <model name='Mailbox'>
        <pose>-7.99778 -2.23277 -9e-06 5e-06 -6e-06 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-7.99778 -2.23277 -9e-06 5e-06 -6e-06 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 -9.8 0 -0 0</acceleration>
          <wrench>0 0 -9.8 0 -0 0</wrench>
        </link>
      </model>
      <model name='bookshelf'>
        <pose>-2.62891 5.09124 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-2.62891 5.09124 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='cabinet'>
        <pose>-1.23232 -1.68825 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='cabinet_bottom_plate'>
          <pose>-1.23232 -1.68825 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='cafe.world'>
        <pose>-0.490708 0.112147 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='Wall_10'>
          <pose>4.38429 -6.31285 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_12'>
          <pose>6.35929 -6.31285 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_13'>
          <pose>7.03429 -4.88785 0 0 -0 1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_16'>
          <pose>7.03429 -2.66285 0 0 -0 1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_18'>
          <pose>5.73429 -1.86285 0 0 -0 3.14159</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_20'>
          <pose>4.43429 -0.812853 0 0 -0 1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_22'>
          <pose>4.43429 1.16215 0 0 -0 1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_24'>
          <pose>5.85929 2.08715 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_25'>
          <pose>7.28429 3.38715 0 0 -0 1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_27'>
          <pose>7.28429 5.61215 0 0 -0 1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_28'>
          <pose>2.60929 6.53715 0 0 -0 3.14159</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_30'>
          <pose>-4.74071 6.53715 0 0 -0 3.14159</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_32'>
          <pose>-7.84071 6.53715 0 0 -0 3.14159</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_34'>
          <pose>-8.26571 5.86215 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_36'>
          <pose>-8.26571 3.76215 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_38'>
          <pose>-8.26571 0.787147 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_40'>
          <pose>-8.26571 -2.18785 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_45'>
          <pose>-8.26571 -4.41285 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_47'>
          <pose>-4.25571 -5.21285 0 0 -0 3.14159</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_49'>
          <pose>-2.17571 -1.32285 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_50'>
          <pose>-0.060708 -3.18785 0 0 0 -1.5306</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_52'>
          <pose>-2.64071 5.18715 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_53'>
          <pose>-0.965708 4.38715 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_55'>
          <pose>-0.965708 2.79215 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_60'>
          <pose>-0.965708 1.99715 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_62'>
          <pose>-3.52571 1.99715 0 0 -0 3.14159</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_8'>
          <pose>1.41929 -5.21285 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
        <link name='Wall_9'>
          <pose>3.08429 -5.76285 0 0 0 -1.5708</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='cafe_table'>
        <pose>-4.45682 3.64828 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-4.45682 3.64828 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='cafe_table_0'>
        <pose>4.31475 4.45493 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>4.31475 4.45493 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='cafe_table_1'>
        <pose>-2.33796 -3.31048 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-2.33796 -3.31048 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='first_2015_trash_can'>
        <pose>-6.75454 -4.91099 0.034287 -0.156153 0.022364 -0.027373</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>-6.75454 -4.91099 0.034287 -0.156153 0.022364 -0.027373</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>-0.359488 0.008047 -0.064565 2.91363 -1.27362 -0.541355</acceleration>
          <wrench>-1.7366 0.038871 -0.311896 0 -0 0</wrench>
        </link>
      </model>
      <model name='fountain'>
        <pose>2.74688 0.353258 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>2.74688 0.353258 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='ground_plane'>
        <pose>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <model name='table'>
        <pose>4.66728 -4.06633 0 0 -0 1.57328</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose>4.66728 -4.06633 0 0 -0 1.57328</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>-8.09555 -14.8926 32.7955 0 1.1338 0.620192</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>

