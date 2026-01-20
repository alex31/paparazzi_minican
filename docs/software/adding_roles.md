# How to Add a New Role

The recommended starting point is the built-in template role:
- [COMMON/source/templateRole.hpp](../../COMMON/source/templateRole.hpp)
- [COMMON/source/templateRole.cpp](../../COMMON/source/templateRole.cpp)

Copy these files, rename the class and files, then adapt subscriptions,
resources, and parameters for your new role.

## Checklist

1) Create role files
- Copy [COMMON/source/templateRole.hpp](../../COMMON/source/templateRole.hpp) and
  [COMMON/source/templateRole.cpp](../../COMMON/source/templateRole.cpp)
- Rename to COMMON/source/MyRole.hpp and COMMON/source/MyRole.cpp
- Rename the class (TemplateRole -> MyRole) and update comments

2) Add role parameter(s)
- Add ROLE.my_role to [COMMON/source/nodeParameters.hpp](../../COMMON/source/nodeParameters.hpp)
- Add any runtime parameters (example: role.template.log_every)

3) Add compile-time toggle
- Add #define USE_MY_ROLE true in [COMMON/source/roleConf.h](../../COMMON/source/roleConf.h)

4) Register in UAVCanSlave
- Add the role include in [minican/source/UAVCanSlave.cpp](../../minican/source/UAVCanSlave.cpp)
- Add addRole<MyRole, FixedString("ROLE.my_role")>(); in CANSlave::start()

5) Allocate resources
- In start(), use boardResource.tryAcquire(...) for pins and peripherals
- If MicroCAN uses shared pins, call DynPin::setScenario(...) where needed

6) Subscribe and publish
- Implement subscribe() for UAVCAN message subscriptions
- Implement start() to launch threads, begin hardware, and publish messages

7) Document the role
- Add an entry in [roles.readme.txt](../../roles.readme.txt)
- Update [docs/software/roles/overview.md](roles/overview.md) if user-visible

## How to Add a New Role with an LLM

Recommended workflow:
1) Provide the LLM with
- Desired role behavior (messages in/out, timing, hardware interface)
- Pins/peripherals required
- Parameter names, limits, defaults

2) Ask the LLM to
- Start from [COMMON/source/templateRole.hpp](../../COMMON/source/templateRole.hpp) and
  [COMMON/source/templateRole.cpp](../../COMMON/source/templateRole.cpp)
- Copy/rename the files and class
- Add parameters to [COMMON/source/nodeParameters.hpp](../../COMMON/source/nodeParameters.hpp)
- Add compile-time toggle in [COMMON/source/roleConf.h](../../COMMON/source/roleConf.h)
- Register in [minican/source/UAVCanSlave.cpp](../../minican/source/UAVCanSlave.cpp)
- Add documentation to [roles.readme.txt](../../roles.readme.txt)

3) Review output for
- Correct resource acquisition
- Thread safety
- Parameter naming conventions
- Consistency with existing roles

Example prompt:
```
Create a new role named FooRole for MiniCAN.
It should read a sensor over I2C1 at 50 Hz and publish uavcan.equipment.foo.Bar.
Use parameter ROLE.foo to enable it and role.foo.sensor_id to set sensor ID.
Add resource allocation for I2C1 and PB07/PA15 pins.
Register it in UAVCanSlave and document it in [roles.readme.txt](../../roles.readme.txt).
```
