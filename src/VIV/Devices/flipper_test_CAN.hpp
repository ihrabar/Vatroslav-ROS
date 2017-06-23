
#include "flipper_test.hpp"

namespace CanROS_UNO {

//! Blocking communication implementation.
/*!

 */
    class CommBlocking : public Communication {
    public:

        //! Constructor
        /*!

         */
        CommBlocking(const CommPar &par) : param_(par),
                                           impl_(CommImpl::Create(par)) {

        }

        //! Open the communication channel.
        /*!

         */
        virtual bool Open(void) {
            return true;
        }

        //! Close the communication channel.
        /*!

         */
        virtual bool Close(void) {
            return true;
        }

        //! Send data.
        /*!
            @param msg	Message to send.
         */
        virtual bool Send(const CommMsg &msg) {
            ROS_INFO("flipperTest CAN send");
            std::cout << "flipperTest CAN send" << std::endl;
            return Can_ROS_UNO_2::SendV(msg);
        }

        //! Receive data.
        /*!
            @param timeout	Amount of time to wait for the incoming message, in
                            usec (?)
            @return The received message.
         */
        virtual bool Receive(CommMsg &msg, unsigned short timeout) {
            bool pom = Can_ROS_UNO_2::ReceiveV(msg, timeout);
            return pom;
        }

        //! Get communication parameters.
        /*!

         */
        virtual const CommPar &Params(void) {
            return impl_.Params();
        }

    private:

        // Reference to the acutal communication channel.
        CommImpl &impl_;
        // Communication channel parameters.
        CommPar param_;

    };
}
