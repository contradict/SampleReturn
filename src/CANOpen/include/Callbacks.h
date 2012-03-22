#ifndef __CALLBACKS_H__
#define __CALLBACKS_H__
namespace CANOpen {

template <class from_class> class CallbackObject {
    public:
        typedef void (TransferCallbackReceiver::*CallbackFunction)(from_class &cls);
        CallbackObject(TransferCallbackReceiver *r,
                       CallbackFunction fn,
                       CallbackFunction efn=NULL
                       ) :
            r(r), fn(fn), efn(efn) {};
        CallbackObject() :
            r(NULL),
            fn(NULL),
            efn(NULL) {};

        virtual void operator()(from_class &cls)
        {
            if(r != NULL && fn != NULL) ((*r).*fn)(cls);
        };

        virtual void error(from_class &cls)
        {
            if(r != NULL && efn != NULL) ((*r).*efn)(cls);
        };


    private:
        TransferCallbackReceiver *r;
        CallbackFunction fn;
        CallbackFunction efn;
};
}
#endif // __CALLBACKS_H__
