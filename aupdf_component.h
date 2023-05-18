// aupdf_component.h
// COMPANY:   Continental Automotive
// COMPONENT: FP AUPDF

#ifndef AUPDF_COMPONENT_H
#define AUPDF_COMPONENT_H

#include "aupdf/cem_outputs.h"
#include "aupdf/dynamic_environment.h"
#include "aupdf/ego_motion_at_cem_output.h"
#include "aupdf/parking_slot_detection_output.h"
#include "aupdf/pcl_output.h"
#include "aupdf/sef_output.h"
#include "aupdf_cfg.h"
#include "cem/algo_data_time_stamp_t.h"
#include "cem/algo_signal_state_t.h"
#include "cem/aupdf/aupdf_interface.h"
#include "cem/aupdf/aupdf_callback.h"
#include "cem/pcl_delimiter_t.h"
#include "cem/pfs_output_t.h"
#include "cem/sef_output_t.h"
#include "cem/static_element.h"
#include "cem/tp_t_object_list.h"
#include "com/com_signal_state_t.h"
#include "eco/component_execution_mode.h"
#include "eco/system_services.h"

#include <cstring> // for memset

// PRQA S 2128 ++
// 2019-04-01; uidr4619
// summary: Msg(3:2127) Overriding non-pure base class virtual function.
//          MISRA - C++ Rule 10 - 3 - 1
// reason:  Functions are part of the framework. They have an implementation to avoid undefined functions in components

namespace cem
{
  namespace aupdf
  {
    // TODO: to be moved to  may be separate file
    struct TimestampsOfLastUpdate
    {
      AlgoDataTimeStamp_t sef{};
      AlgoDataTimeStamp_t sgf{};
      AlgoDataTimeStamp_t pfs{};
      AlgoDataTimeStamp_t eml{};
      AlgoDataTimeStamp_t dyn{};
    };

    class AupdfComponentImpl : public AupdfInterface
    {
      private:
        ////    Attributes    ////
        // @brief Flag will be true only in case services used in component
        //        base will be available
        bool m_validServiceInit;

        /// @copydoc ComponentInterface::init()
        bool init(void);

        /// @copydoc ComponentInterface::suspend()
        bool suspend(void);

        /// @copydoc ComponentInterface::resume()
        bool resume(void);

        /// @copydoc ComponentInterface::shutdown()
        bool shutdown(void);

        /// @copydoc ComponentInterface::setExecutionMode(ComponentExecutionMode)
        bool setExecutionMode(const eco::ComponentExecutionMode executionMode);

        /// @copydoc ComponentInterface::execute()
        bool execute(const uint8 runable);

        template<config::StaticObjectsSource source>
        bool validateStaticObjectsInput();

        template<typename OutputType>
        void mergeOutput(cem::SefOutput_t& result, const OutputType& sefOutput);

        template<typename OutputType>
        bool validateElementForFusion(const cem::StaticElement&);

        template<config::StaticObjectsSource source>
        void fillStaticObjects(::aupdf::CemOutputs& cemOutputs);

        void fillCemOutputHeader(::aupdf::CemOutputs & cemOutputs, bool bEmlValid, const ::cem::EgoVehicleKinematicsQueue &inputEml);

        /// @brief Initialize the last time stamps to 0
        ///
        /// @param[in] void
        ///
        void initLastTimestamps();

        /// @brief Validates input from SEF component
        ///
        /// @param[in] inputSef
        ///
        /// @return
        ///    - True ,if Validation success
        ///    - False if Validation not succesfull
        template<config::StaticObjectsSource source, typename T>
        bool validateStaticObjectProviderInput(const T& staticObjectProvider);

        /// @brief Validates input from PFS component
        ///
        /// @param[in] inputPfs
        ///
        /// @return
        ///    - True ,if Validation success
        ///    - False if Validation not succesfull
        bool validatePfsInput(const cem::PfsOutput_t& inputPfs);

        /// @brief Validates input from EML component
        ///
        /// @param[in] inputEml
        ///
        /// @return
        ///    - True ,if Validation success
        ///    - False if Validation not succesfull
        bool validateEmlInput(const cem::EgoVehicleKinematicsQueue& inputEml);

        /// @brief Validates input from VAL component
        ///
        /// @param[in] inputVal
        ///
        /// @return
        ///    - True ,if Validation success
        ///    - False if Validation not succesfull
        bool validateDynInput(const cem::TP_t_ObjectList& inputDyn);

        /// @brief Validates Out dated input
        ///
        /// @param[in] newTimestamp
        ///
        /// @param[in] oldTimestamp
        ///
        /// @return
        ///    - True ,if newTimestamp >= oldTimestamp
        ///    - False if newTimestamp < oldTimestamp
        static bool validateTimestamp(const AlgoDataTimeStamp_t& newTimestamp, AlgoDataTimeStamp_t& oldTimestamp);

        ///
        /// Validates the signal state
        ///
        /// @param[in] AlgoSignalState_t signal state
        /// @return
        ///    - True ,if sigState != AL_SIG_STATE_INVALID
        ///    - False if sigState == AL_SIG_STATE_INVALID
        static bool validateSigstate(const AlgoSignalState_t sigState);

        /// @brief Convert the data in new SEF output format to aupdf format.
        ///
        /// @param[in]  cem::SefOutput_t                        inputSef
        /// @param[out] cem::aupdf::SefOutput_t                 outputAupdfFormat
        ///
        template<typename T>
        static void convertSefToAupdfFormat(::aupdf::SefOutput& outputAupdfFormat, const T& staticObjectProvider);

        /// @brief Convert the data in new PFS/PCL output format to aupdf format.
        ///
        /// @param[in]  cem::PfsOutput_t                        inputPfs
        /// @param[out] cem::aupdf::CEM_PclOutput               outputAupdfFormat
        ///
        static void convertPclToAupdfFormat(::aupdf::PclOutput& outputAupdfFormat, const cem::PfsOutput_t& inputPfs);

        /// @brief Convert 1 delimiter to CEM output format
        ///
        /// @param[in]  in                        inputPfs
        /// @param[out] out               outputAupdfFormat
        ///
        static void convertPclDelimiter( const cem::PclDelimiter_t& in, ::aupdf::PclDelimiter_t& out);

        /// @brief Convert the data in new PFS/PSD output format to aupdf format.
        ///
        /// @param[in]  cem::PfsOutput_t                        inputPfs
        /// @param[out] cem::aupdf::ParkingSlotDetectionOutput  outputAupdfFormat
        ///
        static void convertPsdToAupdfFormat(::aupdf::ParkingSlotDetectionOutput& outputAupdfFormat, const cem::PfsOutput_t& inputPfs);

        /// @brief Convert the data in new EML output format to aupdf format.
        ///
        /// @param[in]  cem::EgoVehicleKinematicsQueue          inputEml
        /// @param[out] cem::aupdf::EgoVehicleKinematicsQueue   outputAupdfFormat
        ///
        static void convertEmlToAupdfFormat(::aupdf::EgoMotionAtCemOutput& outputAupdfFormat, const cem::EgoVehicleKinematicsQueue& inputEml);

        static void convertTPF2ToAupdfFormat(::aupdf::DynamicEnvironment& dynObjOutput, const ::cem::TP_t_ObjectList& dynObjInput);

        static ::com::ComSignalState_t convertSigState(const cem::AlgoSignalState_t& cemSigState);

        TimestampsOfLastUpdate m_timestampsOfLastUpdate; ///< holds the last timestamps read from the signal headers

        AupdfCallback& m_callback; ///< The reference to the Aupdf callback.

        ::eco::SystemServices& m_systemService; ///< The reference to the system services.

        ::eco::ComponentExecutionMode m_executionMode; ///< The current execution mode.

      public:
        /// The parametrized constructor which initializes the member of the class.
          AupdfComponentImpl(AupdfArguments& arguments);
    };
    template <typename DATA_STRUCT>
    void initializeDataStruct(DATA_STRUCT& data)
    {
      static_cast<void>(memset(&data, 0, sizeof(data)));
    }
  }
}
#endif
