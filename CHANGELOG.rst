^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mqtt_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2021-01-29)
------------------
* changed version number
* example now also has DynamicServer example
  and further examples of qos and latch
* action server launch file is working
  with prefix and dynamic server
* DynamicServer implementation working
* added launching own MQTT server
* removed
* intial actionlib mqtt setup
* support for latched (retained) topics and QoS
* ignore vscode
* changed maintainer
* fix install target for directories
* Contributors: Marc Hanheide

1.2.2 (2021-03-11)
------------------
* Merge pull request `#4 <https://github.com/LCAS/mqtt_bridge/issues/4>`_ from LCAS/futures
  simple threading solution
* simple threading solution
* Contributors: Marc Hanheide

1.2.1 (2021-03-10)
------------------
* Merge pull request `#2 <https://github.com/LCAS/mqtt_bridge/issues/2>`_ from francescodelduchetto/avoid-loops
  Avoid loops in connection with the server
* do not start a new bridge in the Server if the bridge with the reverse connection is already running bcs it create loops in ros messages
* Update README.md
* improved documentation
* Contributors: Marc Hanheide, francescodelduchetto

1.2.0 (2021-01-30)
------------------
* added LocalServiceProxy
* renamed to RemoteService
* Contributors: Marc Hanheide

1.1.0 (2021-01-30)
------------------
* Merge pull request `#1 <https://github.com/LCAS/mqtt_bridge/issues/1>`_ from LCAS/remoteserver
  Remoteserver implementation complete
* implementation of RemoteServer working
* WIP: towards a remote server call
* Contributors: Marc Hanheide

1.0.1 (2021-01-29)
------------------
* missing install target
* Merge branch 'pr_upstream' into melodic-devel
* example now also has DynamicServer example
  and further examples of qos and latch
* action server launch file is working
  with prefix and dynamic server
* DynamicServer implementation working
* added launching own MQTT server
* removed
* intial actionlib mqtt setup
* support for latched (retained) topics and QoS
* Contributors: Marc Hanheide

1.0.0 (2021-01-29)
------------------
* 0.2.1
* changelog
* changed version number
* example now also has DynamicServer example
  and further examples of qos and latch
* action server launch file is working
  with prefix and dynamic server
* DynamicServer implementation working
* added launching own MQTT server
* removed
* intial actionlib mqtt setup
* support for latched (retained) topics and QoS
* ignore vscode
* changed maintainer
* fix install target for directories
* Contributors: Marc Hanheide

0.1.8 (2020-12-31)
------------------
* configure rostest and run it on circleci (`#41 <https://github.com/groove-x/mqtt_bridge/issues/41>`_)
* split requirements.txt (`#38 <https://github.com/groove-x/mqtt_bridge/issues/38>`_)
* add unittests (`#37 <https://github.com/groove-x/mqtt_bridge/issues/37>`_)
* Fix 'install_requires' warning when building with --install (`#32 <https://github.com/groove-x/mqtt_bridge/issues/32>`_)
* Contributors: Jacob Seibert, Junya Hayashi

0.1.7 (2020-06-21)
------------------
* Fix mqtt subscribing to private path (`#27 <https://github.com/groove-x/mqtt_bridge/issues/27>`_)
* Fix frequency limit (`#26 <https://github.com/groove-x/mqtt_bridge/issues/26>`_)
* Add bson module in requirements.txt (`#10 <https://github.com/groove-x/mqtt_bridge/issues/10>`_)
* Fix Bridge not to fall when ros msg cannot be created (`#4 <https://github.com/groove-x/mqtt_bridge/issues/4>`_)
* Contributors: 5tan, Junya Hayashi, Tomas Cernik, Yuma Mihira, kapilPython

0.1.6 (2017-11-10)
------------------
* fix if frequency is none (`#2 <https://github.com/groove-x/mqtt_bridge/issues/2>`_)

0.1.5 (2016-12-07)
------------------
* Update configurations

0.1.0 (2016-12-04)
------------------
* Initial Release
