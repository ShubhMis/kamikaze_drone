INSTALLATION OF GAZEBO HARMONIC IN UBUNTU
IN TERMINAL:
    Run these commands:

[[sudo apt update]] --> Before installing anything, Ubuntu must know what packages and versions are available.
                    --> apt → Ubuntu package manager.
                        update → Downloads the latest list of software packages from configured repositories.
                    --> It does NOT install anything, it only refreshes package information.    

[[sudo apt install curl]] --> curl is used to download files from the internet via URL.(Here it will be used to download the Gazebo repository key).
[[lsb-release gnupg]]     --> This command tells which Ubuntu version you are using.


                           

Add the Gazebo key:

sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/gazebo-archive-keyring.gpg

Add repository:

echo "deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable \
$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

Update again:

sudo apt update
✅ Step 2 — Install Gazebo Harmonic

Now install:

sudo apt install gz-harmonic

This installs:

✔️ Physics
✔️ Rendering
✔️ GUI
✔️ Sensors
✔️ Simulation engine

✅ Step 3 — Test Installation

Run:

gz sim

If installed correctly → Gazebo GUI will open 🎉

Test with a world:

gz sim shapes.sdf
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AFTER INSTALLLATION GO TO THE DIRECTORY AND FOR RUNNING USE THE FOLLOWING COMMANDS:
    gz sim building_robot.sdf
    cd sdf_files/


AFTER THIS INSTALLATION OF ROS2 JAZZY WILL COME THE STEPS ARE:
    sudo apt update
    sudo apt install ros-jazzy-ros-gz


Key PUBLISHER PLUGIN i.e using KEYS:
       <!-- Moving right-->
        <plugin filename="gz-sim-triggered-publisher-system"
                name="gz::sim::systems::TriggeredPublisher">
            <input type="gz.msgs.Int32" topic="/keyboard/keypress">
                <match field="data">16777236</match>
            </input>
            <output type="gz.msgs.Twist" topic="/cmd_vel">
                linear: {x: 0.0}, angular: {z: -0.5}
            </output>
        </plugin>
        <!-- Moving left-->
        <plugin filename="gz-sim-triggered-publisher-system"
                name="gz::sim::systems::TriggeredPublisher">
            <input type="gz.msgs.Int32" topic="/keyboard/keypress">
                <match field="data">16777234</match>
            </input>
            <output type="gz.msgs.Twist" topic="/cmd_vel">
                linear: {x: 0.0}, angular: {z: 0.5}
            </output>
        </plugin> 

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                                                                COMMANDS HOW IT'S WORKING
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.0}"
```

---

### Word by Word

**`gz`**
```
The Gazebo command line tool.
Entry point for all gazebo terminal interactions.
```

**`topic`**
```
Subcommand of gz.
Tells gz we want to interact with a topic (communication channel).
Other subcommands exist like:
    gz model   → interact with models
    gz sim     → interact with simulation
    gz service → interact with services
```

**`-t`**
```
Flag meaning "target topic"
Specifies WHICH topic to publish to.
```

**`/cmd_vel`**
```
The topic name.
/ → root level topic (like a file path)
cmd_vel → command velocity
This is where DiffDrive plugin listens for motion commands.
```

**`-m`**
```
Flag meaning "message type"
Tells gz WHAT FORMAT the message will be in.
```

**`gz.msgs.Twist`**
```
The message type name.
gz.msgs → gazebo messages library
Twist   → the specific message type for velocity commands
Contains two parts:
    linear  → straight line motion
    angular → rotational motion
```

**`-p`**
```
Flag meaning "payload"
The actual data/values to send in the message.
```

**`"linear: {x: 0.5}, angular: {z: 0.0}"`**
```
The payload content.

linear: {x: 0.5}
    └── move forward at 0.5 m/s
        x: positive → forward
        x: negative → backward

angular: {z: 0.0}
    └── no rotation
        z: positive → turn left
        z: negative → turn right
        z: 0.0      → go straight
```

---

## Full picture
```
gz        topic        -t         /cmd_vel      -m          gz.msgs.Twist    -p        "linear: {x: 0.5}, angular: {z: 0.0}"
↓           ↓           ↓            ↓            ↓               ↓            ↓                      ↓
gazebo    interact    target       topic         message        Twist          payload              move forward
tool      with        topic        name          type           format                              at 0.5 m/s
          topics                                flag                                                no rotation
```

---

## Analogy
Think of it like sending a letter:
```
gz topic     → post office
-t /cmd_vel  → address (where to deliver)
-m Twist     → envelope type (what format)
-p "..."     → letter content (actual instruction)

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                                                                    END
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

commands for running:
getting data of imu
    gz topic -e -t /imu
getting wall touched data:
    gz topic -e -t /wall/touched
getting odometry data:
    gz topic -e -t /odom
    

