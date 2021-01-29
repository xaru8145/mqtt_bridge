# -*- coding: utf-8 -*-
from __future__ import absolute_import

from abc import ABCMeta, abstractmethod

import inject
import paho.mqtt.client as mqtt
import rospy

from .util import lookup_object, extract_values, populate_instance
from threading  import Condition

def create_bridge(factory, **kwargs):
    u""" bridge generator function

    :param (str|class) factory: Bridge class
    :param (str|class) msg_type: ROS message type
    :param str topic_from: incoming topic path
    :param str topic_to: outgoing topic path
    :param (float|None) frequency: publish frequency
    :return Bridge: bridge object
    """
    if isinstance(factory, basestring):
        factory = lookup_object(factory)
    if not issubclass(factory, Bridge):
        raise ValueError("factory should be Bridge subclass")
    return factory(**kwargs)


class Bridge(object):
    u""" Bridge base class

    :param mqtt.Client _mqtt_client: MQTT client
    :param _serialize: message serialize callable
    :param _deserialize: message deserialize callable
    """
    __metaclass__ = ABCMeta

    _mqtt_client = inject.attr(mqtt.Client)
    _serialize = inject.attr('serializer')
    _deserialize = inject.attr('deserializer')
    _extract_private_path = inject.attr('mqtt_private_path_extractor')


class DynamicBridgeServer(Bridge):
    u""" Bridge from ROS topic to MQTT

    :param str topic_from: incoming ROS topic path
    :param str topic_to: outgoing MQTT topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    :param bool latched: retain the last message on the MQTT topic (default: False)
    :param int qos: MQTT quality of service (default: 0, max: 2)
    """

    def __init__(self, control_topic="__dynamic_server", ):
        self._control_topic = control_topic + '/topic/#'
        self._mqtt_client.subscribe(self._control_topic, qos=2)
        self._mqtt_client.message_callback_add(self._control_topic, self._callback_mqtt)
        self._bridges = set([])
        rospy.loginfo('DynamicBridgeServer started on control topic %s' % control_topic)

    def _callback_mqtt(self, client, userdata, mqtt_msg):
        u""" callback from MQTT

        :param mqtt.Client client: MQTT client used in connection
        :param userdata: user defined data
        :param mqtt.MQTTMessage mqtt_msg: MQTT message
        """
        rospy.loginfo("MQTT received from {}".format(mqtt_msg.topic))
        msg_dict = self._deserialize(mqtt_msg.payload)

        if msg_dict['op'] == 'mqtt2ros_subscribe':
            rospy.loginfo("forward mqtt topic to ros %s" % (
                msg_dict['args']))
            self._bridges.add(MqttToRosBridge(
                **msg_dict['args'])
            )

        if msg_dict['op'] == 'ros2mqtt_subscribe':
            rospy.loginfo("forward ros topic to mqtt %s" % (
                msg_dict['args']))
            self._bridges.add(RosToMqttBridge(
                **msg_dict['args'])
            )


class RosToMqttBridge(Bridge):
    u""" Bridge from ROS topic to MQTT

    :param str topic_from: incoming ROS topic path
    :param str topic_to: outgoing MQTT topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    :param bool latched: retain the last message on the MQTT topic (default: False)
    :param int qos: MQTT quality of service (default: 0, max: 2)
    """

    def __init__(self, topic_from, topic_to, msg_type, frequency=None, latched=False, qos=0):
        self._topic_from = topic_from
        self._topic_to = self._extract_private_path(topic_to)
        self._last_published = rospy.get_time()
        self._interval = 0 if frequency is None else 1.0 / frequency
        self._latched = latched
        self._qos = qos
        if isinstance(msg_type, basestring):
            msg_type = lookup_object(msg_type)
        if not issubclass(msg_type, rospy.Message):
            raise TypeError(
                "msg_type should be rospy.Message instance or its string"
                "reprensentation")

        rospy.Subscriber(topic_from, msg_type, self._callback_ros)

    def _callback_ros(self, msg):
        rospy.logdebug("ROS received from {}".format(self._topic_from))
        now = rospy.get_time()
        if now - self._last_published >= self._interval:
            self._publish(msg)
            self._last_published = now

    def _publish(self, msg):
        payload = bytearray(self._serialize(extract_values(msg)))
        self._mqtt_client.publish(
            topic=self._topic_to, payload=payload,
            qos=self._qos, retain=self._latched)


class MqttToRosBridge(Bridge):
    u""" Bridge from MQTT to ROS topic

    :param str topic_from: incoming MQTT topic path
    :param str topic_to: outgoing ROS topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    :param int queue_size: ROS publisher's queue size (default: 10)
    :param bool latch: latch the ROS topic (default: False)
    :param int qos: MQTT quality of service (default: 0, max: 2)
    """

    def __init__(self, topic_from, topic_to, msg_type, frequency=None,
                 queue_size=10, latched=False, qos=0):
        self._topic_from = self._extract_private_path(topic_from)
        self._topic_to = topic_to
        if isinstance(msg_type, basestring):
            msg_type = lookup_object(msg_type)
        if not issubclass(msg_type, rospy.Message):
            raise TypeError(
                "msg_type should be rospy.Message instance or its string"
                "reprensentation")
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._latched = latched
        self._qos = qos
        self._last_published = rospy.get_time()
        self._interval = None if frequency is None else 1.0 / frequency
        # Adding the correct topic to subscribe to
        self._mqtt_client.subscribe(self._topic_from, qos=self._qos)
        self._mqtt_client.message_callback_add(self._topic_from, self._callback_mqtt)
        self._publisher = rospy.Publisher(
            self._topic_to, self._msg_type, queue_size=self._queue_size, latch=self._latched)

    def _callback_mqtt(self, client, userdata, mqtt_msg):
        u""" callback from MQTT

        :param mqtt.Client client: MQTT client used in connection
        :param userdata: user defined data
        :param mqtt.MQTTMessage mqtt_msg: MQTT message
        """
        rospy.logdebug("MQTT received from {}".format(mqtt_msg.topic))
        now = rospy.get_time()

        if self._interval is None or now - self._last_published >= self._interval:
            try:
                ros_msg = self._create_ros_message(mqtt_msg)
                self._publisher.publish(ros_msg)
                self._last_published = now
            except Exception as e:
                rospy.logerr(e)

    def _create_ros_message(self, mqtt_msg):
        u""" create ROS message from MQTT payload

        :param mqtt.Message mqtt_msg: MQTT Message
        :return rospy.Message: ROS Message
        """
        msg_dict = self._deserialize(mqtt_msg.payload)
        return populate_instance(msg_dict, self._msg_type())

class SubscribeBridge(MqttToRosBridge):

    def __init__(self, topic_from, topic_to, msg_type, control_topic="__dynamic_server", frequency=None, latched=False, qos=0):
        self._control_topic = control_topic + '/topic/' + topic_from.replace('/', '_')
        self._mqtt_topic = control_topic + '_DATA_' + (topic_from + "_TO_" + topic_to).replace('/','_')
        super(SubscribeBridge, self).__init__(self._mqtt_topic, topic_to, msg_type, frequency, latched, qos)

        rospy.loginfo('SubscribeBridge: subscribe ROS topic %s to topic %s via MQTT %s' %
            (topic_from, topic_to, self._mqtt_topic)
        )

        cmd = {
            'op': 'ros2mqtt_subscribe',
            'args': {
                'topic_from': topic_from, 
                'topic_to': self._mqtt_topic,
                'msg_type': msg_type,
                'frequency': frequency,
                'latched': latched,
                'qos': qos
            }
        }
        payload = bytearray(self._serialize(cmd))
        self._mqtt_client.publish(
            topic=self._control_topic, payload=payload,
            qos=2, retain=True)


class PublishBridge(RosToMqttBridge):

    def __init__(self, topic_from, topic_to, msg_type, control_topic="__dynamic_server", frequency=None, latched=False, qos=0):
        self._control_topic = control_topic + '/topic/' + topic_to.replace('/', '_')
        self._mqtt_topic = control_topic + '_DATA_' + (topic_from + "_TO_" + topic_to).replace('/','_')
        super(PublishBridge, self).__init__(topic_from, self._mqtt_topic, msg_type, frequency, latched, qos)

        rospy.loginfo('PublishBridge: publish from ROS topic %s to topic %s via MQTT %s' %
            (topic_from, topic_to, self._mqtt_topic)
        )

        cmd = {
            'op': 'mqtt2ros_subscribe',
            'args': {
                'topic_from': self._mqtt_topic,
                'topic_to': topic_to,
                'msg_type': msg_type,
                'frequency': frequency,
                'latched': latched,
                'qos': qos
            }
        }
        payload = bytearray(self._serialize(cmd))
        self._mqtt_client.publish(
            topic=self._control_topic, payload=payload,
            qos=2, retain=True)


class RemoteServer(Bridge):

    def __init__(self, local_server, remote_server, srv_type, control_topic="__remote_server"):
        self._local_server = local_server
        self._remote_server = remote_server
        self._control_topic = control_topic
        self._mqtt_topic_request = self._control_topic + '/service/request/' + (local_server + "_TO_" + remote_server).replace('/','_')
        self._mqtt_topic_response = self._control_topic + '/service/response/' + (local_server + "_TO_" + remote_server).replace('/','_')

        self._srv_type_name = srv_type
        self._srv_type = lookup_object(self._srv_type_name)

        self._responses = {}
        self._id_counter = 0

        # Adding the correct topic to subscribe to
        self._mqtt_client.subscribe(self._mqtt_topic_response, qos=2)
        self._mqtt_client.message_callback_add(self._mqtt_topic_response, self._callback_mqtt)

        self._serviceproxy = rospy.Service(self._local_server, self._srv_type, self._ros_handler)

        self._condition = Condition()

    def _next_id(self):
        id = self._id_counter
        self._id_counter += 1
        return id

    def _ros_handler(self, req):
        rospy.loginfo('local service %s called.' % self._local_server)
    # generate a unique ID
        request_id = "service_request:" + self._local_server + ":" + str(self._next_id())
        # build a request to send to the external client
        request_message = {
            "op": "call_service",
            "id": request_id,
            "type": self._srv_type_name,
            "service": self._remote_server,
            "args": extract_values(req)
        }

        payload = bytearray(self._serialize(request_message))
        self._mqtt_client.publish(
            topic=self._mqtt_topic_request, payload=payload,
            qos=2, retain=False)
    
        # wait for a response
        while not rospy.is_shutdown() and request_id not in self._responses.keys():
            with self._condition:
                self._condition.wait(1)  # check for shutdown every 1 second 

        resp = self._responses[request_id]
        del self._responses[request_id]
        return resp
            

    def _callback_mqtt(self, client, userdata, mqtt_msg):
        rospy.loginfo('got response from remote service via mqtt for call to %s -> %s'
            % (self._local_server, self._remote_server)
        )
        response_message = self._deserialize(mqtt_msg.payload)
        response_type = lookup_object(self._srv_type_name+"Response")
        r = response_type()
        populate_instance(response_message['response'], r)
        with self._condition:
            self._responses[response_message['id']] = r
            self._condition.notifyAll()


__all__ = [
    'create_bridge', 'Bridge', 'RosToMqttBridge', 'MqttToRosBridge', 
    'DynamicBridgeServer', 'SubscribeBridge', 'PublishBridge', 'RemoteServer']
