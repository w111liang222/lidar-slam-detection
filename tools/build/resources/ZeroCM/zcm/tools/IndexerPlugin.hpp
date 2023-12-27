#pragma once

#include <string>
#include <set>
#include <cstdint>

#include "zcm/zcm-cpp.hpp"
#include "zcm/json/json.h"

//
// Remember you must inherit from this class and implement your functions
// inside your cpp file. That way you can then compile a shared library of your
// plugins.
//
// REPEAT: ALL FUNCTIONS MUST BE *DEFINED* OUTSIDE OF CLASS *DECLARATION*
//
// Do this:
//
// class CustomPlugin : IndexerPlugin
// {
//     static IndexerPlugin* makeIndexerPlugin();
// };
// IndexerPlugin* CustomPlugin::makeIndexerPlugin() { return new CustomPlugin(); }
//
//
// Not This:
//
// class CustomPlugin : IndexerPlugin
// {
//     static IndexerPlugin* makeIndexerPlugin() { return new CustomPlugin(); }
// };
//
//
// Note that an hpp file is not even required. You only need a cpp file containing
// your custom plugin. Then compile it into a shared library by doing the following:
//
// g++ -std=c++11 -fPIC -shared CustomPlugin.cpp -o plugins.so
//

namespace zcm {

class IndexerPlugin
{
  public:
    // Must declare the following function for your plugin. Unable to enforce
    // declaration of static functions in inheritance, so this API trusts you
    // to define this function for your type
    static IndexerPlugin* makeIndexerPlugin();

    virtual ~IndexerPlugin();

    // This defines the key of the object you will be passed in the functions
    // below as pluginIndex. Further explanation below.
    virtual std::string name() const;

    // Returns a set of names of other plugins that this plugin depends on.
    // Ie if a custom plugin depends on the timestamp plugin's output, it would
    // return {"timestamp"} and no indexing functions would be called on this
    // plugin until the "timestamp" plugin finished its indexing
    // Specifying "timestamp" as a dependency allows this plugin to use the
    // timestamp-indexed data while performing its own indexing operations
    // In other words, returning {"timestamp"} ensures that the index passed
    // into the following functions will contain the entire (not partial)
    // "timestamp" index. The full index passed into the following functions
    // is just for your reference, your indexEvent function will still be called
    // on every single event in the log in the order in which they are stored in
    // the log.
    virtual std::set<std::string> dependsOn() const;

    // Do anything that your plugin requires doing before the indexing process
    // starts but after all dependencies have run
    //
    // the index argument refers to the entirety of the index created thus far.
    // If you had specifided "timestamp" as a dependency of this plugin, index
    // might look like this:
    //
    // {
    //     "timestamp" : {
    //         "IMAGES" : {
    //             "image_t" : [
    //                 "0",
    //                 "1001000",
    //                 "2002000",
    //                 ...
    //                 "37037000"
    //             ]
    //         },
    //         "BEACON_COORDS" : {
    //             "beacon_t" : [
    //                 "1000000",
    //                 "1000100",
    //                 "1000200",
    //                 "1000300",
    //                 ...
    //                 "37036900"
    //             ]
    //         }
    //     },
    //     "custom plugin" : {
    //     }
    // }
    //
    // pluginIndex would be the object in json that you will modify to create
    // your index. In the case above, pluginIndex would be empty when passed
    // into this function:
    //
    //     "custom plugin" : {
    //     }
    //
    // return true from this if you would like the below indexEvent function to
    // be called on every event in the log. returning false from here would
    // skip the log traversal step that calls indexEvent on each event and would
    // skip straight to tear down
    //
    virtual bool setUp(const zcm::Json::Value& index,
                       zcm::Json::Value& pluginIndex,
                       zcm::LogFile& log);

    // pluginIndex is the index you should modify. It is your json object that
    // will be passed back to this function every time the function is called.
    // This function will be called on every event in the log
    //
    // index is the entire json object containing the output of every plugin run
    // so far
    //
    // channel is the channel name of the event
    //
    // typeName is the name of the zcmtype encoded inside the event
    //
    // offset is the index into the eventlog where this event is stored
    // each susbequent indexEvent call will be called with a larger offset
    // than the last. In other words, "offset" is monotonically increasing
    // across indexEvent calls
    //
    // timestamp is the timestamp of the event. Not to be confused with any
    // fields contained in the zcmtype message itself. This is the timestamp
    // of the event as reported by the transport that originally received it.
    //
    // hash is the hash of the type encoded inside the event
    //
    // data and datalen is the payload of the event
    // To recover the zcmtype message from these arguments, simply call:
    //
    //  if (hash == msg_t::getHash()) {
    //      msg_t msg;
    //      msg.decode(data, 0, datalen);
    //  }
    //
    // The default plugin indexes in the following way. You might want to
    // follow suit but that decision is left completely up to you.
    //
    // pluginIndex[channel][typeName].append(offset);
    //
    //
    virtual void indexEvent(const zcm::Json::Value& index,
                            zcm::Json::Value& pluginIndex,
                            std::string channel,
                            std::string typeName,
                            off_t offset,
                            uint64_t timestamp,
                            int64_t hash,
                            const uint8_t* data,
                            int32_t datalen);

    // Do anything that your plugin requires doing before the indexer exits
    // If your data needs to be sorted, do so here
    virtual void tearDown(const zcm::Json::Value& index,
                          zcm::Json::Value& pluginIndex,
                          zcm::LogFile& log);
};

}
