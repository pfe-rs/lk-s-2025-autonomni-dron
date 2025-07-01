import subprocess
import shlex

def spawn_box(mass, x, y, z):
    sdf = f'''<sdf version="1.6">
  <model name="kutija_{mass}kg">
    <pose>{x} {y} {z} 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{mass}</mass>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>1 1 1</size></box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Red</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''

    # Escape quotes for shell argument
    sdf_escaped = shlex.quote(sdf)

    cmd = f'gz service -s /world/default/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 300 --req sdf: {sdf_escaped}'

    print("Executing:", cmd)
    subprocess.run(cmd, shell=True, check=True)

# Primer poziva
spawn_box(10, 0, 0, 0.5)
spawn_box(50, 2, 0, 0.5)
spawn_box(200, 4, 0, 0.5)
