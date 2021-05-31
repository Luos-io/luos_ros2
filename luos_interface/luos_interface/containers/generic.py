class LuosGenericPublisher(object):
    QUEUE_SIZE = 1
    def __init__(self, node, container, rate, variables, events, aggregates):
        self._node = node
        self._container = container
        self._publishers = {}
        self._subscribers = {}
        self._timer = None
        self._rate = rate
        self.variables = variables    # Dict {name: ROSType}
        self.events = events          # Dict {name: ROSType}
        self.aggregates = aggregates  # Dict {name: ROSType}

        # None case is autorised for mardown doc generation
        if node is None and container is None and rate is None: return

        # Open publishers for Luos variables
        for variable, info in self.variables.items():
            topic_root = [container.alias, "variables", variable]
            if "read_type" in info:
                topic = "/".join(topic_root + ["read"])
                self._publishers[variable] = {
                    "topic": topic,
                    "type": info["read_type"],
                    "pub": self._node.create_publisher(info["read_type"], topic, self.QUEUE_SIZE)
                }
            if "write_type" in info:
                topic = "/".join(topic_root + ["write"])
                callback = lambda msg, variable=variable: self._subscription_callback(msg, variable)
                self._subscribers[variable] = {
                    "topic": topic,
                    "type": info["write_type"],
                    "callback": callback,
                }
                self._node.create_subscription(
                    info["write_type"],
                    topic,
                    callback,
                    self.QUEUE_SIZE
                )

        # Open publishers for aggregated Luos variables converted into ROS standard messages
        for aggregate, info in self.aggregates.items():
            type = info["type"]
            topic = "/".join([container.alias, aggregate])
            serialize = info["serialize"]
            self._publishers[aggregate] = {
                "topic": topic,
                "type": type,
                "pub": self._node.create_publisher(type, topic, self.QUEUE_SIZE)
            }

        # Open publishers for Luos events
        for event, info in self.events.items():
            type = info["type"]
            topic = "/".join([container.alias, "events", event])
            serialize = info["serialize"]
            self._publishers[event] = {
                "topic": topic,
                "type": type,
                "pub": self._node.create_publisher(type, topic, self.QUEUE_SIZE)
            }
            container.add_callback(event, lambda e: self._publishers[event]["pub"].publish(serialize(container, e)))

        # Start the timer for variables and aggregates
        self._timer = self._node.create_timer(1./self._rate, self._timer_callback)

    def _subscription_callback(self, msg, variable):
        # A "write" message is incoming, it is transferred to Luos containers
        deserialize = self.variables[variable]["deserialize"]
        setattr(self._container, variable, deserialize(msg))

    def _timer_callback(self):
        # "read" messages are all sent at a specific rate
        # Publish variables on a regular basis (with container-agnostic serializers e.g. Float32, UInt32...)
        for variable, info in self.variables.items():
            serialize = info["serialize"]
            self._publishers[variable]["pub"].publish(serialize(getattr(self._container, variable)))
        
        # Publish aggregates on a regular basis (with container-dependent serializers)
        for aggregate, info in self.aggregates.items():
            serialize = info["serialize"]
            self._publishers[aggregate]["pub"].publish(serialize(self._container))

    def get_markdown_doc(self):
        from io import StringIO
        from std_msgs.msg import Bool
        from luos_interface.containers.deserializers import deserializeBool

        def long_type(type):
            split = str(type).split('\'')[1].split('.')
            return split[0] + "/msg/" + split[-1]

        doc = StringIO()

        for variable in self.variables:
            if "read_type" in self.variables[variable]:
                doc.writelines(["| /mod/variables/{}/read | {}".format(variable, long_type(self.variables[variable]["read_type"])), "\n"])
            if "write_type" in self.variables[variable]:
                doc.writelines(["| /mod/variables/{}/write | {}".format(variable, long_type(self.variables[variable]["write_type"])), "\n"])
        
        for event in self.events:
                doc.writelines(["| /mod/events/{} | {}".format(event, long_type(self.events[event]["type"])), "\n"])
        
        for aggregate in self.aggregates:
                doc.writelines(["| /mod/{} | {}".format(aggregate, long_type(self.aggregates[aggregate]["type"])), "\n"])
        
        return doc.getvalue()
