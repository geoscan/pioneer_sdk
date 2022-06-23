# About `mavsub` package

"Mavsub" stands for "MAVLink subprotocols".

Despite the fact that `pymavlink` provides quite comprehensive set of marshalling, unmarshalling, and communication facilities, some [subprotocols](http://mavlink.io/en/services/) require implementing rather complicated message exchange sequences which better be encapsulated.
MAVLink FTP subprotocol is one of many.
This directory contains a set of complexity-encapsulating modules offering convenient APIs while implementing various MAVLink subprotocols under the hood.

You are encouraged to create such encapsulations for unifying semantically and pragmatically similar MAVLink communication sequences, such as those pertaining to autonomous navigation, even when those are not described by MAVLink standard as parts of some subprotocol.

When you use this approach for implementing some *vendor-specific* functionality, please make sure that your choice for entity naming reflects the fact that the implementation you create is not generic.

