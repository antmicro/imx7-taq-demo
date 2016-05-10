TAQ Control Protocol
=======================

Introduction
------------

TAQ may be controlled from Linux over the RPMsg mechanism. The TAQ Control Protocol is implemented on the top of the TTY interface, which in turn uses the RPMsg.
To control TAQ from Linux (which runs on CA7 part of the iMX7), a few requirements should be met:

* RPMsg mechanism enabled in the Linux kernel
* compiled and loaded ``imx_rpmsg_tty`` kernel module

When the ``imx_rpmsg_tty`` module is loaded, the ``/dev/ttyRPMSG`` interface will appear. It acts as a serial interface. For an example of the TAQ Control Protocol implementation, refer to the ``TAQ-commander`` application. It is a C application which implements most parts of the protocol.

General command structure
-------------------------
``<!,?><command>[:<value>][:<value>]``

where:

* ``<!,?>`` indicates SET (``!``) or GET (``?``) command
* ``<command>`` command (case sensitive)
* ``<value>`` numerical value(s) for SET command, for GET command it can be any value (ignored)

General response structure
--------------------------
``<!,?><command>:<result>``

where:

* ``<!,?>`` and ``<command>`` are the same as in request,
* ``<result>`` is requested value (for GET) or ``ok``/``err`` (for SET)

Commands
--------
Movements
+++++++++
``!move:<value>``

moves TAQ forth or back with the speed ``<value>``, where ``<value>`` is within the range of (-10; 10)

``!turnLeft:<value>``
``!turnRight:<value>``

turns TAQ right or left with speed ``<value>``, where ``<value>`` is within the range of (0; 30)

``!stop``

immediately stops all movements of TAQ

Activity
++++++++

``!servo:<id>:<value>``

set the servo ``<number>`` position to ``<value>``, where:

* ``<id>`` is in range 1-4
* ``<value>`` is in range (0; 180)

Status
++++++

``?angle``

returns the current lean angle of TAQ:

* 0 - the robot is in vertical position
* >0 - the robot leans to the front
* <0 - the robot leans to the back

``?speedLeft``
``?speedRight``

returns the current speed of each motor (>0 forward move, <0 backward move)

``?distance``

returns value from distance sensor [cm]

``?voltage``

returns battery voltage multiplied by 100
