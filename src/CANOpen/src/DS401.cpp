#include <SDO.h>
namespace CANOpen {

class DS401 : public DS301 {

    public:
        void initialize(void);

    private:
        void status_callback(PDO &pdo);

};

}

#include <DS301.h>
namespace CANOpen {

void DS401::initialize() {
    std::tr1::shared_ptr<PDO> status_pdo(new PDO());


}


}
