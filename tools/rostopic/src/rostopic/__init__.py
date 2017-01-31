# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division, print_function

NAME='rostopic'

import os
import sys
import math
import socket
import time
try:
    from xmlrpc.client import Fault
except ImportError:
    from xmlrpclib import Fault

import rosgraph
import rospy

from rostopic.exceptions import ROSTopicException


class Verb(object):
    """
    Base class for the verbs which implement rostopic's functionality.
    """

    def get_arg_parser(self):
        raise NotImplementedError

    def invoke(self, parser_result):
        raise NotImplementedError

def get_plugins():
    from rospkg import RosPack
    from rospkg.common import PACKAGE_FILE
    from catkin_pkg.package import parse_package, InvalidPackage
    from xml.etree import ElementTree

    rp = RosPack()
    plugins = []
    for package_name in rp.list():
        package_path = rp.get_path(package_name)
        package_file_path = os.path.join(package_path, PACKAGE_FILE)
        if os.path.isfile(package_file_path):
            package = parse_package(package_file_path)
            for export in package.exports:
                if export.tagname != 'rostopic':
                    continue
                plugin_xml_path = export.attributes['plugin'].replace('${prefix}', package_path)
                plugin_xml_tree = ElementTree.parse(plugin_xml_path)
                for plugin_xml_class in plugin_xml_tree.findall('./class'):
                    plugins.append((
                        plugin_xml_class.attrib['name'],
                        plugin_xml_class.attrib['type'],
                        plugin_xml_class.attrib['base_class_type'],
                        plugin_xml_class.find('description').text))
    return plugins

def print_usage(plugins):
    verb_plugins = [ v for v in plugins if v[2] == 'rostopic::Verb' ]
    print("""rostopic is a command-line tool for printing information about ROS Topics.

Available verbs:""")

    for name, type, base_class_type, description in verb_plugins:
        print("\trostopic %s\t%s" % (name, description))

    print("""
Type rostopic <verb> -h for more detailed usage, e.g. 'rostopic echo -h'
""")
    sys.exit(getattr(os, 'EX_USAGE', 1))


#from rostopic.verbs.echo import _rostopic_cmd_echo
#from rostopic.verbs.hz import _rostopic_cmd_hz
#from rostopic.verbs.type import _rostopic_cmd_type
#from rostopic.verbs.list import List
#from rostopic.verbs.info import _rostopic_cmd_info
#from rostopic.verbs.pub import _rostopic_cmd_pub
#from rostopic.verbs.bw import _rostopic_cmd_bw
#from rostopic.verbs.find import _rostopic_cmd_find
#from rostopic.verbs.delay import _rostopic_cmd_delay


def rostopicmain(argv=None):
    import rosbag
    if argv is None:
        argv = sys.argv
    # filter out remapping arguments in case we are being invoked via roslaunch
    argv = rospy.myargv(argv)
    plugins = get_plugins()

    # process argv
    if len(argv) == 1:
        print_usage(plugins)
        return

    try:


        command = argv[1]
        parser_result = verb.get_arg_parser().parse_args(argv[2:])
        verb.invoke(parser_result)

        if command == 'echo':
            _rostopic_cmd_echo(argv)
        elif command == 'hz':
            _rostopic_cmd_hz(argv)
        elif command == 'type':
            _rostopic_cmd_type(argv)
        elif command == 'list':


        elif command == 'info':
            _rostopic_cmd_info(argv)
        elif command == 'pub':
            _rostopic_cmd_pub(argv)
        elif command == 'bw':
            _rostopic_cmd_bw(argv)
        elif command == 'find':
            _rostopic_cmd_find(argv)
        elif command == 'delay':
            _rostopic_cmd_delay(argv)
        else:
            _fullusage()
    except socket.error:
        sys.stderr.write("Network communication failed. Most likely failed to communicate with master.\n")
        sys.exit(1)
    except rosbag.ROSBagException as e:
        sys.stderr.write("ERROR: unable to use bag file: %s\n"%str(e))
        sys.exit(1)
    except rosgraph.MasterException as e:
        # mainly for invalid master URI/rosgraph.masterapi
        sys.stderr.write("ERROR: %s\n"%str(e))
        sys.exit(1)
    except ROSTopicException as e:
        sys.stderr.write("ERROR: %s\n"%str(e))
        sys.exit(1)
    except KeyboardInterrupt: pass
    except rospy.ROSInterruptException: pass
