var zcm = (function(){

    var imported = document.createElement('script');
    imported.src = `zcm/socket.io.js`;
    document.head.appendChild(imported);

    function create(socketOptions = {})
    {
        var socket = io({ ...socketOptions, path: "/zcm" });

        var subscriptions = {};

        // key is zcmtype name
        var zcmtypes = null;

        socket.on('server-to-client', function (data) {
            var subId = data.subId;
            if (subId in subscriptions)
                subscriptions[subId].callback(data.channel, data.msg);
        });

        socket.on('zcmtypes', function (data) { zcmtypes = data; });

        function getZcmtypes(key)
        { return key ? JSON.parse(JSON.stringify(zcmtypes[key])) : zcmtypes; }

        /**
         * Publishes a message on the given channel of the specified zcmtype
         * @param {string} channel - the zcm channel to publish on
         * @param {string} type - the zcmtype of messages on the channel (must be a generated
         *                        type from zcmtypes.js)
         * @param {zcmtype} msg - a decoded zcmtype (from the generated types in zcmtypes.js)
         */
        function publish(channel, msg) {
            socket.emit('client-to-server', { channel : channel,
                                              msg     : msg });
        }

        /**
         * Subscribes to zcm messages on the given channel of the specified zcmtype.
         * @param {string} channel - the zcm channel to subscribe to
         * @param {string} type - the zcmtype of messages on the channel (must be a generated
         *                        type from zcmtypes.js). Passing null yields an untyped
         *                        subscription.
         * @param {dispatchDecodedCallback} handler - handler for received messages
         */
        function subscribe(channel, type, handler, successCb) {
            socket.emit("subscribe", { channel : channel, type : type },
                        function (subId) {
                            subscriptions[subId] = { callback : handler,
                                                     channel  : channel,
                                                     type     : type };
                            if (successCb) successCb(subId);
                        });
        }

        /**
         * Unsubscribes from the zcm messages on the given channel
         * @param {int} subId - the subscription tag to unsubscribe from
         */
        function unsubscribe(subId, successCb) {
            if (subId in subscriptions) {
                socket.emit("unsubscribe", subId,
                            function () {
                                delete subscriptions[subId];
                                if (successCb) successCb(true);
                            });
                return;
            }
            console.log("No subscription found, cannot unsubscribe");
            if (successCb) successCb(false);
        }

        /**
         * Forces all incoming and outgoing messages to be flushed to their
         * handlers / to the transport.
         * @params {doneCb} doneCb - callback for successful flush
         */
        function flush(doneCb) {
            socket.emit("flush", doneCb);
        }

        /**
         * Pauses transport publishing and message dispatch
         */
        function pause(cb)
        {
            socket.emit("pause", cb);
        }

        /**
         * Resumes transport publishing and message dispatch
         */
        function resume(cb)
        {
            socket.emit("resume", cb);
        }

        /**
         * Sets the recv and send queue sizes within zcm
         */
        function setQueueSize(sz, cb)
        {
            socket.emit('setQueueSize', sz, cb);
        }

        return {
            publish:        publish,
            subscribe:      subscribe,
            unsubscribe:    unsubscribe,
            flush:          flush,
            pause:          pause,
            resume:         resume,
            setQueueSize:   setQueueSize,
            getZcmtypes:    getZcmtypes,
        };
    }

    return {
        create: create
    };
})();
