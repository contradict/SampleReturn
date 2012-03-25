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
        CallbackObject(std::tr1::shared_ptr<TransferCallbackReceiver> sr,
                       CallbackFunction fn,
                       CallbackFunction efn=NULL
                       ) :
            r(NULL), sr(sr), fn(fn), efn(efn) {};
         CallbackObject() :
            r(NULL),
            fn(NULL),
            efn(NULL) {};

        virtual void operator()(from_class &cls)
        {
            if(r != NULL && fn != NULL) ((*r).*fn)(cls);
            else if(sr != NULL && fn != NULL) ((*sr).*fn)(cls);
        };

        virtual void error(from_class &cls)
        {
            if(r != NULL && efn != NULL) ((*r).*efn)(cls);
            else if(sr != NULL && efn != NULL) ((*sr).*efn)(cls);
        };


    private:
        std::tr1::shared_ptr<TransferCallbackReceiver> sr;
        TransferCallbackReceiver *r;
        CallbackFunction fn;
        CallbackFunction efn;
};
}
#endif // __CALLBACKS_H__
