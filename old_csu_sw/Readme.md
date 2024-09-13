# CSU EtherCAT Test Software

This software was quickly developed to test the EtherCAT communication between the CSU and the EtherCAT slaves. The software is written in Python and uses the pySOEM library to communicate with the EtherCAT slaves. This is a wrapper for the SOEM library written in C.

Slave register writes are abstracted away in the EPOS4.py file. It would probably be a good idea to make these into a class but such is life. ¯\\\_(◕‿◕✿)_/¯ 

LongTermTesting might be a good place to start for anyone diving into this.

SOEM_exploration might also be good for a more stripped back thing. 