# Roboost Firmware Collection

This repository contains the codebase for multiple components of the Roboost robotic system.
The naming of each part of the system is based on regions of the brain that are responsible for similar tasks.

Goals of the Roboost project:
- Minimize the cost of robotic development with ROS
- Provide an easy-to-use base for more complex systems
- Configurable for different types of robots

More information can be found on the dedicated blog posts:
- [Roboost V2 Robot Summary](https://jakobfriedl.tech/project-summary/)
- [Roboost V2 Showcase](https://technologiehub.at/project-posts/roboost-v2/)
- ...

## Entorhinal Cortex

The Entorhinal Cortex is a PlatformIO project that contains the firmware for an ESP32. It is responsible for reading navigation-specific sensor data (for example LiDAR and IMU sensors) and publishing it onto the corresponding ROS topics.

More information can be found in the following blog post:
- [Roboost Entorhinal Cortex Summary](https://technologiehub.at/project-posts/roboost-entorhinal-cortex/)

#### Biology of the Entorhinal Cortex - Explanation by ChatGPT:
>"The entorhinal cortex is a part of the brain located in the medial temporal lobe, which is an important region involved in memory processes and navigation.
>
>Imagine it like a central hub or a middle-man in your brain's network. Information about what you see, hear, and experience flows into the entorhinal cortex. From there, the entorhinal cortex processes and integrates this sensory information and sends it to the hippocampus, another area of the brain that's vital for memory storage and consolidation.
>
>In essence, the entorhinal cortex helps to sort out and filter the data before it goes into the 'memory bank' (the hippocampus).
>
>It also plays a crucial role in spatial memory and navigation. Think of it like the brain's internal GPS system. Within the entorhinal cortex are cells known as 'grid cells'. These grid cells activate in specific patterns as you move around, creating a sort of mental map of your environment. This is how you remember how to navigate familiar places.
>
>Just as you as a computer engineering student might think of a server or router as managing and routing data traffic, you can think of the entorhinal cortex as a brain's manager for incoming sensory information and spatial navigation.
>
>In fact, some researchers have suggested that the way the entorhinal cortex and hippocampus interact in forming and retrieving memories bears some similarities to how computers manage and retrieve data, though of course, the human brain is far more complex and flexible than any computer system."

## Primary Motor Cortex

The Primary Motor Cortex contains code related to the motor control of the robotic system. It is also based on a PlatformIO project for an ESP32 and listens to the /cmd_vel topic in the ROS network. The received messages then are converted into individual motor speeds which then are used to control the given motors.

More information can be found in the following blog post:
- [Roboost Primary Motor Cortex Summary](https://technologiehub.at/project-posts/roboost-primary-motor-cortex/)

#### Biology of the Primary Motor Cortex - Explanation by ChatGPT:
>"The primary motor cortex is like the command center that controls voluntary movements.
>
>The primary motor cortex is located in the frontal lobe of the brain, towards the back. It's like a programmer sitting at a keyboard, sending out commands. These commands aren't lines of code, though, but signals that direct your body to move.
>
>Here's how it works: When you decide to make a voluntary movement, like picking up a coffee mug, this information is processed in the frontal areas of your brain. Once the decision is made, the signal gets passed to the primary motor cortex, which maps out a plan for how to execute that movement. This plan includes which muscles need to be activated, in what order, and how strongly.
>
>This mapped out plan is then sent down the spinal cord to the muscles that are involved in the movement. It's like the primary motor cortex writes a piece of code (in the form of an electrical signal), then sends it down the line to be executed by the appropriate 'hardware' (your muscles).
>
>Another interesting feature is that different parts of the primary motor cortex are responsible for different parts of the body, and the amount of cortex devoted to each body part is proportional to how fine the control of movement is in that part. This is known as the motor homunculus. For example, a lot of the primary motor cortex is dedicated to the hands and fingers, which require very precise control, compared to, say, the back muscles.
>
>So, in the language of computer engineering, you could say that the primary motor cortex is a kind of sophisticated, dynamic programming system for movement, generating complex 'scripts' that control the 'hardware' of your body."


## TODO
- Add references for PCBs and used electrical components
