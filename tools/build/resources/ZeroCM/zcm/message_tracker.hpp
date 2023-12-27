#pragma once

#include <iostream>
#include <cmath>
#include <cstdint>
#include <string>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <typeinfo>
#include <cxxabi.h>
#include <sys/time.h>
#include <stdarg.h>
#include <functional>
#include <tuple>
#include <type_traits>

#include <zcm/zcm-cpp.hpp>
#include <zcm/util/Filter.hpp>

static bool __ZCM_DEBUG_ENABLED__ = (NULL != getenv("ZCM_DEBUG"));
#define ZCM_DEBUG(...) \
    do { \
        if (__ZCM_DEBUG_ENABLED__) \
            __ZCM_PRINT_OBFUSCATE__(std::cout, "ZCM-DEBUG: ", __VA_ARGS__, '\n'); \
    } while(0)

// Forward declaration of the test
class MessageTrackerTest;

namespace zcm {

template <typename T>
class Tracker
{
  public:
    // You must free the memory passed into this callback
    // This callback is not guaranteed to be called on every message.
    // It will only be called again on a new message that is received *after*
    // the last call to this callback has returned.
    typedef std::function<void (T* msg, uint64_t utime, void* usr)> callback;

    typedef T ZcmType;

  protected:
    virtual uint64_t getMsgUtime(const T* msg) const { return UINT64_MAX; }

    // The returned value must be "new" in all cases
    virtual T* interpolate(uint64_t utimeTarget,
                           const T* A, uint64_t utimeA,
                           const T* B, uint64_t utimeB) const
    {
        return utimeTarget - utimeA < utimeB - utimeTarget ? new T(*A) : new T(*B);
    }

  private:
    // *****************************************************************************
    // SFINAE gets hard to read sometimes. The point of these 2 classes is to determine
    // if the input type has a "utime" or a "getUtime" field.
    template <typename MsgType> struct hasUtime {
        struct Fallback { int utime; }; // empty class with member "utime"
        struct Derived : MsgType, Fallback {}; // ambiguity if MsgType has "utime"

        // Struct that takes a type and an instance of that type as an argument
        // We'll use this with type = "pointer to element in class" and the instance
        // to be the "utime" field
        template <typename MemberType, MemberType> struct Check;
        // neg test : specifically checking to see if the "utime" field in the type
        //            we are testing actually points to the "utime" field in Fallback
        template <typename TypeToTest>
        static char (&test(Check<int Fallback::*, &TypeToTest::utime>*))[1];
        // pos test : really just a fallback to the negative test
        template <typename TypeToTest>
        static char (&test(...))[2];

        static constexpr bool present = sizeof(test<Derived>(0)) == 2;
    };
    template <typename MsgType> struct hasUtimeFn {
        struct Fallback { int getUtime; }; // empty class with member "getUtime"
        struct Derived : MsgType, Fallback {}; // ambiguity if MsgType has "getUtime"

        // Struct that takes a type and an instance of that type as an argument
        // We'll use this with type = "pointer to element in class" and the instance
        // to be the "getUtime" field
        template <typename MemberType, MemberType> struct Check;
        // neg test : specifically checking to see if the "getUtime" field in the type
        //            we are testing actually points to the "getUtime" field in Fallback
        template <typename TypeToTest>
        static char (&test(Check<int Fallback::*, &TypeToTest::getUtime>*))[1];
        // pos test : really just a fallback to the negative test
        template <typename TypeToTest>
        static char (&test(...))[2];

        // using 0 as an argument here because it can cast to a Check<...>*
        static constexpr bool present = sizeof(test<Derived>(0)) == 2;
    };

    // We'l use the above SFINAE member finders to determine if we should
    // use the
    template<typename F, bool, bool>
    struct MsgWithUtime;

    template<typename F>
    struct MsgWithUtime<F, true, false> : public F
    {
        MsgWithUtime(const F& msg, uint64_t utime) : F(msg) {}
        MsgWithUtime(const MsgWithUtime& msg) : F(msg) {}
        uint64_t getUtime() const { return F::utime; }
        static uint64_t getMsgUtime(const F& msg) { return msg.utime; }
        virtual ~MsgWithUtime() {}
    };

    template<typename F>
    struct MsgWithUtime<F, true, true> : public F
    {
        MsgWithUtime(const F& msg, uint64_t utime) : F(msg) {}
        MsgWithUtime(const MsgWithUtime& msg) : F(msg) {}
        static uint64_t getMsgUtime(const F& msg) { return msg.getUtime(); }
        virtual ~MsgWithUtime() {}
    };

    template<typename F>
    struct MsgWithUtime<F, false, true> : public F
    {
        MsgWithUtime(const F& msg, uint64_t utime) : F(msg) {}
        MsgWithUtime(const MsgWithUtime& msg) : F(msg) {}
        static uint64_t getMsgUtime(const F& msg) { return msg.getUtime(); }
        virtual ~MsgWithUtime() {}
    };

    template<typename F>
    struct MsgWithUtime<F, false, false> : public F
    {
      private:
        uint64_t utime;

      public:
        MsgWithUtime(const F& msg, uint64_t utime) : F(msg), utime(utime) {}
        MsgWithUtime(const MsgWithUtime& msg) : F(msg), utime(msg.utime) {}
        uint64_t getUtime() const { return utime; }
        static uint64_t getMsgUtime(const MsgWithUtime& msg) { return msg.utime; }
        static uint64_t getMsgUtime(const F& msg)
        { ZCM_ASSERT(false && "Cannot use this function on types with no utime"); }
        virtual ~MsgWithUtime() {}
    };

    typedef MsgWithUtime<T, hasUtime<T>::present, hasUtimeFn<T>::present> MsgType;

    // *****************************************************************************

    uint64_t maxTimeErr_us;

    // This is only to be used for the callback thread func
    bool done = false;

  public:
    typedef std::deque<MsgType*> ContainerType;

  private:
    ContainerType buf;
    uint64_t lastHostUtime = UINT64_MAX;
    size_t bufMax;
    typedef std::recursive_mutex BufLockType;
    mutable BufLockType bufLock;

    BufLockType callbackLock;
    std::condition_variable_any callbackCv;
    MsgType* callbackMsg = nullptr;
    std::thread *thr = nullptr;
    callback onMsg;
    void* usr;

    Filter hzFilter;
    Filter jitterFilter;

    void callbackThreadFunc()
    {
        std::unique_lock<BufLockType> lk(callbackLock);
        while (!done) {
            callbackCv.wait(lk, [&](){ return callbackMsg || done; });
            if (done) return;
            onMsg(callbackMsg, callbackMsg->getUtime(), usr);
            // Intentionally not deleting callbackMsg as it is the
            // responsibility of the callback to delete the memory
            callbackMsg = nullptr;
        }
    }

  public:
    ///////////////////////////////
    //// Iterator Defn and Ops ////
    ///////////////////////////////

    // Note: Be careful of asynchronous iterator invalidation if using a tracker
    //       subscribed to a running zcm thread.

    typedef typename ContainerType::              iterator               iterator;
    typedef typename ContainerType::        const_iterator         const_iterator;
    typedef typename ContainerType::      reverse_iterator       reverse_iterator;
    typedef typename ContainerType::const_reverse_iterator const_reverse_iterator;

    inline                iterator  begin()       { return buf.  begin(); }
    inline          const_iterator cbegin() const { return buf. cbegin(); }
    inline                iterator    end()       { return buf.    end(); }
    inline          const_iterator   cend() const { return buf.   cend(); }

    inline       reverse_iterator  rbegin()       { return buf. rbegin(); }
    inline const_reverse_iterator crbegin() const { return buf.crbegin(); }
    inline       reverse_iterator    rend()       { return buf.   rend(); }
    inline const_reverse_iterator   crend() const { return buf.  crend(); }

    // Note: you probably want to `delete *iter` before calling erase on it
    template <typename IterType>
    inline IterType erase(IterType iter) { return buf.erase(iter); }

    ///////////////////////////////

    uint64_t getMsgUtime(const MsgType* msg) const
    {
        uint64_t tmp = getMsgUtime((const T*)msg);
        if (tmp != UINT64_MAX) return tmp;
        return msg->getUtime();
    }

    Tracker(double maxTimeErr = 0.25, size_t maxMsgs = 1,
            callback onMsg = callback(), void* usr = nullptr,
            double freqEstConvergenceNumMsgs = 10)
        : maxTimeErr_us(maxTimeErr * 1e6), onMsg(onMsg), usr(usr),
              hzFilter(Filter::convergenceTimeToNatFreq(freqEstConvergenceNumMsgs, 0.8), 0.8),
          jitterFilter(Filter::convergenceTimeToNatFreq(freqEstConvergenceNumMsgs, 1), 1)
    {
        T tmp;
        std::string name = demangle(getType(tmp));

        if (hasUtimeFn<T>::present == true) {
            ZCM_DEBUG("Message trackers using 'getUtime()' function of type ", name);
        } else if (hasUtime<T>::present == true) {
            ZCM_DEBUG("Message trackers using 'utime' field of type ", name);
        } else {
            ZCM_DEBUG("Message trackers using local system receive utime for ",
                      "tracking zcmtype ", name);
        }

        bufMax = maxMsgs;
        assert(maxMsgs > 0 && "Cannot allocate a tracker to track 0 messages");

        if (onMsg) thr = new std::thread(&Tracker<T>::callbackThreadFunc, this);
    }

    virtual ~Tracker()
    {
        if (thr) {
            {
                std::unique_lock<BufLockType> lk(callbackLock);
                done = true;
                callbackCv.notify_all();
            }
            thr->join();
            delete thr;
            if (callbackMsg) delete callbackMsg;
        }

        std::unique_lock<BufLockType> lk(bufLock);
        while (!buf.empty()) {
            MsgType* tmp = buf.front();
            delete tmp;
            buf.pop_front();
        }
    }

    // You must free the memory returned here. This may return nullptr
    T* get() const
    {
        T* ret = nullptr;

        {
            std::unique_lock<BufLockType> lk(bufLock);
            if (!buf.empty()) ret = new T(*buf.back());
        }

        return ret;
    }

    // Same semantics as get()
    virtual T* get(uint64_t utime) const
    {
        std::unique_lock<BufLockType> lk(bufLock);
        return get(utime, buf.begin(), buf.end(), &lk);
    }

    // TODO: Should consider how to allow the user to ask for an extrapolated
    //       message if bracketted messages arent available
    // If you need to have a lock while working with the iterators, pass it in
    // here to have it unlocked once this function is done working with the iterators
    template <class InputIter, typename lockType = std::mutex>
    T* get(uint64_t utime, InputIter first, InputIter last,
           std::unique_lock<lockType>* lk = nullptr) const
    {
        T *m0 = nullptr, *m1 = nullptr; // two poses bracketing the desired utime
        uint64_t m0Utime = 0, m1Utime = UINT64_MAX;

        const T* _m0 = nullptr;
        const T* _m1 = nullptr;

        // The reason why we do a linear search is to support the ability
        // to skip around in logs. We want to support messages with
        // non-monitonically increasing utimes and this is the easiest way.
        // This can be made much faster if needed
        for (auto iter = first; iter != last; ++iter) {
            // Note: This is unsafe unless we rely on the static assert at the beginning of
            //       the function
            auto* m = *iter;
            uint64_t mUtime = getMsgUtime(m);
            if (mUtime == UINT64_MAX) mUtime = MsgType::getMsgUtime(*m);

            if (mUtime <= utime && (_m0 == nullptr || mUtime > m0Utime)) {
                _m0 = m;
                m0Utime = mUtime;
            }

            if (mUtime >= utime && (_m1 == nullptr || mUtime < m1Utime)) {
                _m1 = m;
                m1Utime = mUtime;
            }
        }

        if (_m0 != nullptr) m0 = new T(*_m0);
        if (_m1 != nullptr) m1 = new T(*_m1);

        if (lk && lk->owns_lock()) lk->unlock();

        if (m0 && utime - m0Utime > maxTimeErr_us) {
            delete m0;
            m0 = nullptr;
        }

        if (m1 && m1Utime - utime > maxTimeErr_us) {
            delete m1;
            m1 = nullptr;
        }

        if (m0 && m1) {
            if (m0Utime == m1Utime) {
                delete m1;
                return m0;
            }

            T* elt = interpolate(utime, m0, m0Utime, m1, m1Utime);

            delete m0;
            delete m1;

            return elt;
        }

        if (m0) return m0;
        if (m1) return m1;

        return nullptr;
    }

    // This search is inclusive and can't return a message outside [A,B]
    virtual std::vector<T*> getRange(uint64_t utimeA, uint64_t utimeB) const
    {
        std::unique_lock<BufLockType> lk(bufLock);

        // See reason for linear search given in the get() function
        std::vector<T*> ret;
        for (const MsgType* m : buf) {
            uint64_t mUtime = getMsgUtime(m);
            if (utimeA <= mUtime && mUtime <= utimeB)
                ret.push_back(new T(*m));
        }
        return ret;
    }

    size_t expireBefore(uint64_t utime)
    {
        size_t ret = 0;
        std::unique_lock<BufLockType> lk(bufLock);

        // Expire things that are too old
        while (!buf.empty()) {
            if (getMsgUtime(buf.front()) >= utime) break;
            delete buf.front();
            buf.pop_front();
            ++ret;
        }

        for (auto iter = buf.begin(); iter != buf.end();) {
            if (getMsgUtime(*iter) < utime) {
                delete *iter;
                iter = buf.erase(iter);
                ++ret;
            } else {
                ++iter;
            }
        }

        return ret;
    }

    // hostUtime is only used and required when _msg does not have an
    // internal utime field
    // Returns utime of message
    virtual uint64_t newMsg(const T& _msg, uint64_t hostUtime = UINT64_MAX)
    {
        MsgType* tmp = new MsgType(_msg, hostUtime);
        uint64_t tmpUtime = getMsgUtime(tmp);

        {
            std::unique_lock<BufLockType> lk(bufLock);

            // Expire due to buffer being full
            if (buf.size() == bufMax) {
                MsgType* tmp = buf.front();
                delete tmp;
                buf.pop_front();
            }

            // Run the filter for jitter and frequency
            if (lastHostUtime != UINT64_MAX) {
                if (lastHostUtime > hostUtime) {
                    hzFilter.reset();
                } else {
                    double obs = hostUtime - lastHostUtime;
                    hzFilter(obs, 1);
                    double jitterObs = obs - hzFilter[Filter::LOW_PASS];
                    double jitterObsSq = jitterObs * jitterObs;
                    jitterFilter(jitterObsSq, 1);
                    /*
                    std::cout << hostUtime << ", "
                              << jitterObs << ", "
                              << sqrt(jitterFilter.lowPass()) << ", "
                              << obs << ", "
                              << hzFilter.lowPass()
                              << std::endl;
                    // */
                }
            }

            lastHostUtime = hostUtime;
            buf.push_back(tmp);
        }

        // Dispatch to callback
        if (thr) {
            if (callbackLock.try_lock()) {
                if (callbackMsg) delete callbackMsg;
                callbackMsg = new MsgType(_msg, hostUtime);
                callbackLock.unlock();
                callbackCv.notify_all();
            }
        }

        return tmpUtime;
    }

    double getHz() const
    {
        std::unique_lock<BufLockType> lk(bufLock);
        double lp = hzFilter[Filter::LOW_PASS];
        return lp <= 1e-9 ? -1 : 1e6 / lp;
    }

    uint64_t lastMsgHostUtime() const
    {
        {
            std::unique_lock<BufLockType> lk(bufLock);
            if (!buf.empty()) return lastHostUtime;
        }
        return UINT64_MAX;
    }

    double getJitterUs() const
    {
        std::unique_lock<BufLockType> lk(bufLock);
        return sqrt(jitterFilter[Filter::LOW_PASS]);
    }



  private:
    template<typename F>
    static inline std::string getType(const F t) { return typeid(t).name(); }

    static inline std::string demangle(const std::string& name)
    {
        int status = -4; // some arbitrary value to eliminate the compiler warning
        std::unique_ptr<char, void(*)(void*)> res {
            abi::__cxa_demangle(name.c_str(), NULL, NULL, &status),
            std::free
        };
        return (status==0) ? res.get() : name ;
    }

    static void __ZCM_PRINT_OBFUSCATE__(std::ostream& o) { }

    template<typename First, typename ...Rest>
    static void __ZCM_PRINT_OBFUSCATE__(std::ostream& o, First && first, Rest && ...rest)
    {
        o << std::forward<First>(first);
        __ZCM_PRINT_OBFUSCATE__(o, std::forward<Rest>(rest)...);
    }
};

// Note: Remember you have to call the Tracker<T> constructor directly
//       yourself due to the virtual inheritance below!
template <typename T>
class MessageTracker : public virtual Tracker<T>
{
  private:
    zcm::ZCM* zcmLocal = nullptr;
    zcm::Subscription *s = nullptr;

    MessageTracker() {}

    void _handle(const zcm::ReceiveBuffer* rbuf, const std::string& chan, const T* _msg)
    { this->handle(_msg, rbuf->recv_utime); }

  protected:
    virtual uint64_t handle(const T* _msg, uint64_t hostUtime = UINT64_MAX)
    { return this->newMsg(*_msg, hostUtime); }

  public:
    MessageTracker(zcm::ZCM* zcmLocal, const std::string& channel,
                   double maxTimeErr = 0.25, size_t maxMsgs = 1,
                   typename Tracker<T>::callback onMsg = typename Tracker<T>::callback(),
                   void* usr = nullptr,
                   double freqEstConvergenceNumMsgs = 20)
        : Tracker<T>(maxTimeErr, maxMsgs, onMsg, usr, freqEstConvergenceNumMsgs),
          zcmLocal(zcmLocal)
    {
        if (zcmLocal && channel != "")
            s = zcmLocal->subscribe(channel, &MessageTracker<T>::_handle, this);
    }

    virtual ~MessageTracker()
    { if (s) zcmLocal->unsubscribe(s); }
};

// This class will attempt to synchronize messages of type Type1Tracker::ZcmType to
// messages of type Type2Tracker::ZcmType.
//
// details:
//
// Whenever a Synchronizer receives a message of Type1Tracker::ZcmType
// it checks to see if there is a message of type Type2Tracker::ZcmType that has a utime no
// earlier than the received message of ZcmType1. If there is a message of
// type Type2Tracker::ZcmType that satisfies this criteria, then `get` will return a valid pair
// with the original message and the result of `t2.get(msg1.utime)`. If there is
// not a later message, this class will wait until a message of type Type2Tracker::ZcmType is
// received that is no earlier than the message of Type1Tracker::ZcmType.
//
// TODO: Make a SynchronizedTracker class that this class uses.
//       Similar to MessageTracker and Tracker above
//

template <typename Type1Tracker, typename Type2Tracker>
class SynchronizedMessageDispatcher
{

  public:
    typedef std::function<void(typename Type1Tracker::ZcmType*,
                               typename Type2Tracker::ZcmType*,
                               void*)> callback;
  private:
    template <std::size_t... Is>
    struct indices {};

    template <std::size_t N, std::size_t... Is>
    struct build_indices : build_indices<N-1, N-1, Is...> {};

    template <std::size_t... Is>
    struct build_indices<0, Is...> : indices<Is...> {};

    template <typename... Args1, std::size_t... Is1,
              typename... Args2, std::size_t... Is2>
    SynchronizedMessageDispatcher(zcm::ZCM* zcmLocal,
                                  const std::string& channel1,                     size_t maxMsgs1,
                                  const std::string& channel2, double maxTimeErr2, size_t maxMsgs2,
                                  callback onSynchronizedMsg, void* usr,
                                  const std::tuple<Args1...>& args1, const indices<Is1...>& idc1,
                                  const std::tuple<Args2...>& args2, const indices<Is2...>& idc2) :
        t1(zcmLocal, channel1,           0, maxMsgs1, this, std::get<Is1>(args1)...),
        t2(zcmLocal, channel2, maxTimeErr2, maxMsgs2, this, std::get<Is2>(args2)...),
        onSynchronizedMsg(onSynchronizedMsg), usr(usr) {}

    class TrackerOverride1 : public Type1Tracker
    {
      private:
        SynchronizedMessageDispatcher* smt;

      public:
        uint64_t handle(const typename Type1Tracker::ZcmType* _msg,
                        uint64_t hostUtime = UINT64_MAX) override
        {
            uint64_t utime = Type1Tracker::handle(_msg, hostUtime);
            if (this->begin() == this->end()) return utime;
            auto last = std::prev(this->end());
            smt->process(last);
            return utime;
        }

        template <typename... Args>
        TrackerOverride1(zcm::ZCM* zcmLocal, const std::string& channel,
                         double maxTimeErr, size_t maxMsgs,
                         SynchronizedMessageDispatcher* smt,
                         Args&&... args) :
            Tracker<typename Type1Tracker::ZcmType>(maxTimeErr, maxMsgs),
            Type1Tracker(zcmLocal, channel, maxTimeErr, maxMsgs, std::forward<Args>(args)...),
            smt(smt)
        {}
    };

    class TrackerOverride2 : public Type2Tracker
    {
      private:
        SynchronizedMessageDispatcher* smt;

      public:
        uint64_t handle(const typename Type2Tracker::ZcmType* _msg,
                        uint64_t hostUtime = UINT64_MAX) override
        {
            uint64_t utime = Type2Tracker::handle(_msg, hostUtime);
            smt->process(smt->t1.begin());
            return utime;
        }

        template <typename... Args>
        TrackerOverride2(zcm::ZCM* zcmLocal, const std::string& channel,
                         double maxTimeErr, size_t maxMsgs,
                         SynchronizedMessageDispatcher* smt,
                         Args&&... args) :
            Tracker<typename Type2Tracker::ZcmType>(maxTimeErr, maxMsgs),
            Type2Tracker(zcmLocal, channel, maxTimeErr, maxMsgs, std::forward<Args>(args)...),
            smt(smt) {}
    };

    void process(typename Type1Tracker::iterator t1Begin)
    {
        auto it = t2.crbegin();
        if (it == t2.crend()) return;

        uint64_t t2Utime = t2.getMsgUtime(*it);

        for (auto it = t1Begin; it != t1.end();) {
            uint64_t t1Utime = t1.getMsgUtime(*it);
            if (t1Utime <= t2Utime) {
                auto msg2 = t2.get(t1Utime);
                if (msg2) {
                    onSynchronizedMsg(*it, msg2, usr);
                    // Note that we intentionally DO NOT delete *it since we
                    // forfeit this memory to the callback
                    // delete *it;
                    it = t1.erase(it);
                    continue;
                }
            }
            ++it;
        }
    }

    TrackerOverride1 t1;
    TrackerOverride2 t2;

    callback onSynchronizedMsg;
    void* usr;

    static_assert(std::is_base_of<MessageTracker<typename Type1Tracker::ZcmType>,
                                  Type1Tracker>::value,
                  "Tracker type1 must be an extension of MessageTracker<type1>");
    static_assert(std::is_base_of<MessageTracker<typename Type2Tracker::ZcmType>,
                                  Type2Tracker>::value,
                  "Tracker type2 must be an extension of MessageTracker<type2>");

  public:
    template <typename... Args1, std::size_t... Is1,
              typename... Args2, std::size_t... Is2>
    SynchronizedMessageDispatcher(zcm::ZCM* zcmLocal,
                                  const std::string& channel1,                     size_t maxMsgs1,
                                  const std::string& channel2, double maxTimeErr2, size_t maxMsgs2,
                                  callback onSynchronizedMsg, void* usr = nullptr,
                                  const std::tuple<Args1...>& args1 = {},
                                  const std::tuple<Args2...>& args2 = {}) :
        SynchronizedMessageDispatcher(zcmLocal,
                                      channel1,              maxMsgs1,
                                      channel2, maxTimeErr2, maxMsgs2,
                                      onSynchronizedMsg, usr,
                                      args1, build_indices<sizeof...(Args1)>{},
                                      args2, build_indices<sizeof...(Args2)>{}) {}

    void newType1Msg(const typename Type1Tracker::ZcmType* msg)
    {
        t1.handle(msg);
    }

    void newType2Msg(const typename Type2Tracker::ZcmType* msg)
    {
        t2.handle(msg);
    }

    Type1Tracker* getType1Ptr() { return &t1; }
    Type2Tracker* getType2Ptr() { return &t2; }

    friend class ::MessageTrackerTest;
};

}

#undef ZCM_DEBUG
