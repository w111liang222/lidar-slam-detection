/*******************************************************
 * NodeJS FFI bindings to ZCM
 * --------------------------
 * Could/Should be more efficent, but it should
 * suffice in the short-term
 ******************************************************/
var ffi = require('ffi-napi');
var ref = require('ref-napi');
var StructType = require('ref-struct-napi');
var ArrayType = require('ref-array-napi');
var bigint = require('big-integer');
var assert = require('assert');

// Define some types
var voidRef = ref.refType('void')
var charRef = ref.refType('char')

var recvBuf = StructType({
    // Note: it is VERY important that this struct match the zcm_recv_buf_t struct in zcm.h
    utime: ref.types.uint64,
    zcm:   voidRef,
    data:  charRef,
    len:   ref.types.uint32,
});
var recvBufRef = ref.refType(recvBuf);

var subscription = StructType({
    // Note: it is VERY important that this struct match the zcm_sub_t struct in zcm.h
    channel:  ArrayType(ref.types.char),
    callback: voidRef,
    usr:      voidRef,
});
var subscriptionRef = ref.refType(subscription);

// Define our Foreign Function Interface to the zcm library
var libzcm = new ffi.Library('libzcm', {
    'zcm_retcode_name_to_enum': ['int',     ['string']],
    'zcm_create':               ['pointer', ['string']],
    'zcm_destroy':              ['void',    ['pointer']],
    'zcm_publish':              ['int',     ['pointer', 'string', 'pointer', 'int']],
    'zcm_try_subscribe':        ['pointer', ['pointer', 'string', 'pointer', 'pointer']],
    'zcm_try_unsubscribe':      ['int',     ['pointer', 'pointer']],
    'zcm_start':                ['void',    ['pointer']],
    'zcm_try_stop':             ['int',     ['pointer']],
    'zcm_try_flush':            ['int',     ['pointer']],
    'zcm_pause':                ['void',    ['pointer']],
    'zcm_resume':               ['void',    ['pointer']],
    'zcm_try_set_queue_size':   ['int',     ['pointer', 'int']],
    'zcm_write_topology':       ['int',     ['pointer', 'string']],
});

var ZCM_EOK              = libzcm.zcm_retcode_name_to_enum("ZCM_EOK");
var ZCM_EINVALID         = libzcm.zcm_retcode_name_to_enum("ZCM_EINVALID");
var ZCM_EAGAIN           = libzcm.zcm_retcode_name_to_enum("ZCM_EAGAIN");
var ZCM_ECONNECT         = libzcm.zcm_retcode_name_to_enum("ZCM_ECONNECT");
var ZCM_EINTR            = libzcm.zcm_retcode_name_to_enum("ZCM_EINTR");
var ZCM_EUNKNOWN         = libzcm.zcm_retcode_name_to_enum("ZCM_EUNKNOWN");
var ZCM_NUM_RETURN_CODES = libzcm.zcm_retcode_name_to_enum("ZCM_NUM_RETURN_CODES");

exports.ZCM_EOK              = ZCM_EOK;
exports.ZCM_EINVALID         = ZCM_EINVALID;
exports.ZCM_EAGAIN           = ZCM_EAGAIN;
exports.ZCM_ECONNECT         = ZCM_ECONNECT;
exports.ZCM_EINTR            = ZCM_EINTR;
exports.ZCM_EUNKNOWN         = ZCM_EUNKNOWN;
exports.ZCM_NUM_RETURN_CODES = ZCM_NUM_RETURN_CODES;

/**
 * Callback that handles data received on the zcm transport which this program has subscribed to
 * @callback dispatchRawCallback
 * @param {string} channel - the zcm channel
 * @param {array} data - raw data that can be decoded into a zcmtype
 */

/**
 * Callback that handles data received on the zcm transport which this program has subscribed to
 * @callback dispatchDecodedCallback
 * @param {string} channel - the zcm channel
 * @param {zcmtype} msg - a decoded zcmtype
 */

/**
 * Creates a dispatch function that can interface with the ffi library
 * @param {dispatchRawCallback} cb - the js callback function to be linked into the ffi library
 */
function makeDispatcher(cb)
{
    return function (rbuf, channel, usr) {
        var pointerSize = ref.coerceType('size_t').alignment;
        var int64Size   = ref.coerceType('uint64').alignment;
        var int32Size   = ref.coerceType('uint32').alignment;
        var bigEndian   = ref.endianness == 'BE';
        var offset = 0;

        var utime = ref.readUInt64  (rbuf, offset);
        offset += int64Size;

        var zcmPtr = ref.readPointer(rbuf, offset);
        offset += pointerSize;

        var data   = ref.readPointer(rbuf, offset);
        offset += pointerSize;

        var len = 0;
        if (bigEndian == true)
            len = rbuf.readUInt32BE(offset);
        else
            len = rbuf.readUInt32LE(offset);
        offset += int32Size;

        var dataBuf = ref.reinterpret(data, len);
        cb(channel, dataBuf);
    }
}

function zcm(zcmtypes, zcmurl)
{
    const parent = this;

    parent.currSubId = 0;
    parent.subscriptions = {};
    parent.zcmtypeHashMap = {};
    // Note: recursive to handle packages (which are set as objects in the zcmtypes exports)
    function rehashTypes(zcmtypes) {
        for (var type in zcmtypes) {
            if (type == 'getZcmtypes') continue;
            if (typeof(zcmtypes[type]) == 'object') {
                rehashTypes(zcmtypes[type]);
                continue;
            }
            parent.zcmtypeHashMap[zcmtypes[type].__get_hash_recursive()] = zcmtypes[type];
        }
    }
    rehashTypes(zcmtypes);

    parent.z = libzcm.zcm_create(zcmurl);
    if (parent.z.isNull()) {
        return null;
    }

    /**
     * Publishes a zcm message on the created transport
     * @param {string} channel - the zcm channel to publish on
     * @param {string} type - the zcmtype of messages on the channel (must be a generated
     *                        type from zcmtypes.js)
     * @param {zcmtype instance} msg - the decoded message (must be a zcmtype)
     */
    zcm.prototype.publish = function(channel, msg)
    {
        return publish_raw(channel, parent.zcmtypeHashMap[msg.__hash].encode(msg));
    }

    /**
     * Publishes a zcm message on the created transport
     * @param {string} channel - the zcm channel to publish on
     * @param {array} data - the encoded message (use the encode function of a generated zcmtype)
     */
    function publish_raw(channel, data)
    {
        return libzcm.zcm_publish(parent.z, channel, data, data.length);
    }

    /**
     * Subscribes to a zcm channel on the created transport
     * @param {string} channel - the zcm channel to subscribe to
     * @param {string} type - the zcmtype of messages on the channel (must be a generated
     *                        type from zcmtypes.js)
     * @param {dispatchDecodedCallback} cb - callback to handle received messages
     * @param {successCb} successCb - callback for successful subscription
     */
    zcm.prototype.subscribe = function(channel, _type, cb, successCb)
    {
        if (_type) {
            // Note: this lookup is because the type that is given by a client doesn't have
            //       the necessary functions, so we need to look up our complete class here
            var hash = bigint.isInstance(_type.__hash) ?  _type.__hash.toString() : _type.__hash;
            var type = parent.zcmtypeHashMap[hash];
            var sub = subscribe_raw(channel, function (channel, data) {
                var msg = type.decode(data)
                if (msg != null) cb(channel, msg);
            }, successCb);
        } else {
            var sub = subscribe_raw(channel, cb, successCb);
        }
    }

    /**
     * Subscribes to a zcm channel on the created transport
     * @param {string} channel - the zcm channel to subscribe to
     * @param {dispatchRawCallback} cb - callback to handle received messages
     * @param {successCb} successCb - callback for successful subscription
     */
    function subscribe_raw(channel, cb, successCb)
    {
        if (!successCb) assert(false, "subcribe requires a success callback to be specified");
        var dispatcher = makeDispatcher(cb);
        var funcPtr = ffi.Callback('void', [recvBufRef, 'string', 'pointer'], dispatcher);
        setTimeout(function sub() {
            var subs = libzcm.zcm_try_subscribe(parent.z, channel, funcPtr, null);
            if (ref.isNull(subs)) {
                setTimeout(sub, 0);
                return;
            }
            const id = parent.currSubId;
            parent.subscriptions[parent.currSubId] = {
              "id"                : id,
              "subscription"      : subs,
              "nativeCallbackPtr" : funcPtr,
              "dispatcher"        : dispatcher
            }
            parent.currSubId++;
            successCb(parent.subscriptions[id]);
        }, 0);
    }

    /**
     * Unsubscribes from the zcm channel referenced by the given subscription
     * @param {subscriptionRef} subscription - ref to the subscription to be unsubscribed from
     * @param {successCb} successCb - callback for successful unsubscription
     */
    zcm.prototype.unsubscribe = function(sub, successCb)
    {
        if (!(sub.id in parent.subscriptions)) return;
        setTimeout(function unsub() {
            var ret = libzcm.zcm_try_unsubscribe(parent.z, sub.subscription);
            if (ret != ZCM_EOK) {
                setTimeout(unsub, 0);
                return;
            }
            delete parent.subscriptions[sub.id];
            if (successCb) successCb();
        }, 0)
    }

    /**
     * Forces all incoming and outgoing messages to be flushed to their handlers / to the transport.
     * @params {doneCb} doneCb - callback for successful flush
     */
    zcm.prototype.flush = function(doneCb)
    {
        setTimeout(function f() {
            var ret = libzcm.zcm_try_flush(parent.z);
            if (ret != ZCM_EOK) {
                setTimeout(f, 0);
                return;
            }
            if (doneCb) doneCb();
        }, 0)
    }

    /**
     * Starts the zcm internal threads. Called by default on creation
     */
    zcm.prototype.start = function()
    {
        libzcm.zcm_start(parent.z);
    }

    /**
     * Stops the zcm internal threads.
     * @params {stoppedCb} stoppedCb - callback for successful stop
     */
    zcm.prototype.stop = function(stoppedCb)
    {
        setTimeout(function s() {
            var ret = libzcm.zcm_try_stop(parent.z);
            if (ret != ZCM_EOK) {
                setTimeout(s, 0);
                return;
            }
            if (stoppedCb) stoppedCb();
        }, 0)
    }

    /**
     * Pauses transport publishing and message dispatch
     */
    zcm.prototype.pause = function()
    {
        libzcm.zcm_pause(parent.z);
    }

    /**
     * Resumes transport publishing and message dispatch
     */
    zcm.prototype.resume = function()
    {
        libzcm.zcm_resume(parent.z);
    }

    /**
     * Sets the recv and send queue sizes within zcm
     */
    zcm.prototype.setQueueSize = function(sz, cb)
    {
        setTimeout(function s() {
            var ret = libzcm.zcm_try_set_queue_size(parent.z, sz);
            if (ret != ZCM_EOK) {
                setTimeout(s, 0);
                return;
            }
            if (cb) cb();
        }, 0)
    }

    /**
     * Writes the topology index file showing what channels were published and received
     */
    zcm.prototype.writeTopology = function(name)
    {
        libzcm.zcm_write_topology(parent.z, name);
    }

    zcm.prototype.destroy = function()
    {
        libzcm.zcm_destroy(parent.z);
    }

    parent.start();
}

function zcm_create(zcmtypes, zcmurl, http, socketIoOptions = {})
{
    var ret = new zcm(zcmtypes, zcmurl);

    if (http) {
        var io = require('socket.io')(http, { ...socketIoOptions, path: "/zcm" });

        io.on('connection', function (socket) {
            var subscriptions = {};
            var nextSub = 0;
            socket.on('client-to-server', function (data) {
                ret.publish(data.channel, data.msg);
            });
            socket.on('subscribe', function (data, returnSubscription) {
                var subId = nextSub++;
                ret.subscribe(data.channel, data.type, function (channel, msg) {
                    socket.emit('server-to-client', {
                        channel: channel,
                        msg: msg,
                        subId: subId
                    });
                }, function successCb (subscription) {
                    subscriptions[subId] = subscription;
                    returnSubscription(subId);
                });
            });
            socket.on('unsubscribe', function (subId, successCb) {
                if (! (subId in subscriptions)) {
                    successCb();
                    return;
                }
                ret.unsubscribe(subscriptions[subId],
                                function _successCb() {
                                    delete subscriptions[subId];
                                    if (successCb) successCb();
                                });
            });
            socket.on('flush', function (doneCb) {
                ret.flush(doneCb);
            });
            socket.on('pause', function (cb) {
                ret.pause();
                if (cb) cb();
            });
            socket.on('resume', function (cb) {
                ret.resume();
                if (cb) cb();
            });
            socket.on('setQueueSize', function (sz, cb) {
                ret.setQueueSize(sz, cb);
            });
            socket.on('disconnect', function () {
                for (var subId in subscriptions) {
                    ret.unsubscribe(subscriptions[subId]);
                    delete subscriptions[subId];
                }
                nextSub = 0;
            });
            socket.emit('zcmtypes', zcmtypes.getZcmtypes());
        });
    }

    return ret;
}

exports.create = zcm_create;
