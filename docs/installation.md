# Installation

If you've never done it before, setting up a working environment for a robot can be a major headache, so we provide an easy way to set up the working environment with our `a2s` tool.

## 📋 Requirements

- **Operative system**: [Ubuntu 22.04 Desktop](https://releases.ubuntu.com/jammy/)
- **Architecture**: amd64 or arm64

The rationale behind these requirements:
1. The world of robotics runs essentially on GNU/Linux based operating systems, so if you want to develop in the field what better than start using GNU/Linux?
2. We want to enforce Tier 1 support. To minimize dependency problems with packages distributed as pre-compiled binaries we are going to stick to [REP-2000](https://www.ros.org/reps/rep-2000.html#humble-hawksbill-may-2022-may-2027) and only support the above mentioned operating system and architectures.

Comments in possible scenarios:
- Can I use dual-boot? yes.
- Can I use Windows? yes, as long as you use Ubuntu 22.04 with [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install) on Windows 11 with a version equal or higher than 22H2. This is important for rendering Ubuntu graphics applications (like the simulator) on your Windows desktop.
- Can I use a virtual machine? Technically yes but it is very problematic with the graphics drivers and you will experience problems with the simulations so it is not recommended to use virtual machine. But if you still want to try, go ahead and good luck.
- Can I use a Mac: No! 😅.

