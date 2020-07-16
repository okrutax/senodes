

## [About](https://www.senodes.com/docs/sensor-node/introduction/about/ "About")

- [Full Documentation](https://www.senodes.com/docs/sensor-node/ "Full Documentation")
- [Hardware Overview](https://www.senodes.com/docs/sensor-node/introduction/hardware-overview/ "Hardware Overview")
- [SNODE Logic Engine](https://www.senodes.com/docs/sensor-node/introduction/snode-logic-engine/ "SNODE Logic Engine")
- [Get started](https://www.senodes.com/docs/sensor-node/introduction/get-started/ "Get started")

**SNODE** (Sensor Node)  is an open source library that allows to make it easy to build a multi-master serial bus network for hot-plugging nodes with sensors.

**Hardware support**
- **Arduino** ( Nano, Uno, Micro );
- CAN controller ( **MCP2515** );

**IDE**
- PlatformIO

**Source code**

[Download](senodes-master "Download") or Clone the library from GitHub with:

`git clone https://github.com/okrutax/senodes.git`

Communication between nodes is done over the CAN (Controller Area Network) bus. The CAN bus is a communications protocol which has been around for years as a vehicle bus standard designed to allow nodes to communicate with each other without the need for a host.

In an implementation of SNODE each node could have a few sensors. Each sensor of the node is able to send and receive messages to/from the other sensors of nodes over the CAN bus. This means that a node can have several sensors, each of which can be a master or a slave (the sensor mode type is input or output).

On the picture below with the help of arrows is shown schematic connection between sensors (one sensor is controlled by another) of each of the node.

![Topology](https://www.senodes.com/wp-content/uploads/2020/06/Get-Started-Figure-2.svg)

A short video below shows how the SNODE network works.

[![Senodes modules communicate between](https://www.senodes.com/wp-content/uploads/2020/07/4QovW0.gif)](https://youtu.be/tX2KFh9Pr6U)

**SENODES** is set of hexagonal sensors allows to create various inventions using the SNODE library.

The video below shows how Senodes modules communicate between each other using the **SNODE** library.

[![Senodes modules communicate between](https://www.senodes.com/wp-content/uploads/2020/07/P7oK5y.gif)](https://youtu.be/yhjiUp-Ipxg)

Using the SNODE library simplifies the development process, enabling you to build connected devices in minutes and bring your idea to life.

For more information, please refer to [senodes page](https://www.senodes.com/).

Email: robot@boteon.com