// aupdf_component.cpp
// COMPANY:   Continental Automotive
// COMPONENT: FP AUPDF

#include "aupdf_component.h"
#include "aupdf_tpf_obj_converter.h"
#include "aupdf/constants.h"
#include "aupdf/sef_agpelement.h"
#include "aupdf/sef_agpvertex.h"
#include "aupdf/dynamic_environment.h"
#include "aupdf_tpf_obj_converter.h"
#include "cem/aupdf_execution_error_t.h"
#include "cem/sgf_output_t.h"
#include "cem/static_element_category.h"
#include "cem/symbolic_constants.h"
#include "cem/static_element_drivability.h"
#include "cem/e_element_semantic_t.h"
#include "cem/e_element_classification_t.h"
#include "cem/static_element_vertex.h"
#include "cem/cem200_long_mot_states.h"
#include "cem/relation_to_psd_t.h"
#include "cem/obj_number_t.h"
#include "lsm_vedodo/motion_state.h"
#include "lsm_vedodo/direction.h"
#include "eml_access.h"
#include "Cem_Types.h"
#include "cml_ext_cpp.h"

namespace cem
{
  namespace aupdf
  {
    void AupdfComponentImpl::initLastTimestamps() {
      // initializing the last time stamps to 0
      m_timestampsOfLastUpdate = TimestampsOfLastUpdate();
    }

    template<>
    bool AupdfComponentImpl::validateStaticObjectProviderInput<config::StaticObjectsSource::SEF_ONLY>(
        const cem::SefOutput_t& inputStaticObjectProvider) {
      bool isValid = true;

      if (!validateSigstate(inputStaticObjectProvider.signalHeader.eSigStatus)) {
        m_callback.reportError(
            ::cem::AupdfExecutionError_t::AUPDF_ERROR_SEF_INPUT_SIGSTATE_NOT_OK);
        isValid = false;
      }

      if (!validateTimestamp(inputStaticObjectProvider.signalHeader.uiTimeStamp,
        m_timestampsOfLastUpdate.sef))
      {
        m_callback.reportError(
            ::cem::AupdfExecutionError_t::AUPDF_ERROR_SEF_INPUT_OUTDATED);
        isValid = false;
      }

      return isValid;
    }

    template<>
    bool AupdfComponentImpl::validateStaticObjectProviderInput<config::StaticObjectsSource::SGF_ONLY>(
      const cem::SgfOutput_t& inputStaticObjectProvider)
    {
      bool isValid = true;

      if (!validateSigstate(inputStaticObjectProvider.signalHeader.eSigStatus))
      {
        m_callback.reportError(
          ::cem::AupdfExecutionError_t::AUPDF_ERROR_SGF_INPUT_SIGSTATE_NOT_OK);
        isValid = false;
      }

      if (!validateTimestamp(inputStaticObjectProvider.signalHeader.uiTimeStamp,
        m_timestampsOfLastUpdate.sgf))
      {
        m_callback.reportError(
          ::cem::AupdfExecutionError_t::AUPDF_ERROR_SGF_INPUT_OUTDATED);
        isValid = false;
      }

      return isValid;
    }

    bool AupdfComponentImpl::validatePfsInput(
        const cem::PfsOutput_t &inputPfs) {
      bool isValid = true;

      if (!validateSigstate(inputPfs.signalHeader.eSigStatus)) {
        m_callback.reportError(
            ::cem::AupdfExecutionError_t::AUPDF_ERROR_PFS_INPUT_SIGSTATE_NOT_OK);
        isValid = false;
      }

      if (!validateTimestamp(inputPfs.signalHeader.uiTimeStamp,
                             m_timestampsOfLastUpdate.pfs)) {
        m_callback.reportError(
          ::cem::AupdfExecutionError_t::AUPDF_ERROR_PFS_INPUT_OUTDATED);
        isValid = false;
      }

      return isValid;
    }

    bool AupdfComponentImpl::validateEmlInput(
        const cem::EgoVehicleKinematicsQueue &inputEml) {
      bool isValid = true;

      if (!validateSigstate(inputEml.sigHeader.eSigStatus)) {
        m_callback.reportError(
            ::cem::AupdfExecutionError_t::AUPDF_ERROR_EML_INPUT_SIGSTATE_NOT_OK);
        isValid = false;
      }

      if (!validateTimestamp(inputEml.sigHeader.uiTimeStamp,
                             m_timestampsOfLastUpdate.eml)) {
        m_callback.reportError(
            ::cem::AupdfExecutionError_t::AUPDF_ERROR_EML_INPUT_OUTDATED);
        isValid = false;
      }

      return isValid;
    }

    bool AupdfComponentImpl::validateDynInput(
        const cem::TP_t_ObjectList &inputDyn) {
      bool isValid = true;

      if (!validateSigstate(inputDyn.sSigHeader.eSigStatus)) {
        m_callback.reportError(
            ::cem::AupdfExecutionError_t::AUPDF_ERROR_VAL_INPUT_SIGSTATE_NOT_OK);
        isValid = false;
      }

      if (!validateTimestamp(inputDyn.sSigHeader.uiTimeStamp,
                             m_timestampsOfLastUpdate.dyn)) {
        m_callback.reportError(
            ::cem::AupdfExecutionError_t::AUPDF_ERROR_VAL_INPUT_OUTDATED);
        isValid = false;
      }

      return isValid;
    }

    bool AupdfComponentImpl::validateTimestamp(
        const AlgoDataTimeStamp_t &newTimestamp,
        AlgoDataTimeStamp_t &oldTimestamp) {
      bool isValid = true;
      if ((newTimestamp <= oldTimestamp) && (newTimestamp != 0ULL)) {
        isValid = false;
      }
      return isValid;
    }

    bool AupdfComponentImpl::validateSigstate(
        const AlgoSignalState_t sigState) {
      bool isValid = true;
      if (::cem::AlgoSignalState_t::AL_SIG_STATE_INVALID == sigState) {
        isValid = false;
      }
      return isValid;
    }

    AupdfComponentImpl::AupdfComponentImpl(AupdfArguments& arguments)
        : AupdfInterface(), m_validServiceInit(false), m_callback(arguments.callback()),
          m_systemService(arguments.systemServices()), m_executionMode() {}

    bool AupdfComponentImpl::init(void) {
      initLastTimestamps();
      m_validServiceInit = true;
      return m_validServiceInit;
    }

    template<>
    bool AupdfComponentImpl::validateStaticObjectsInput<config::StaticObjectsSource::SEF_ONLY>()
    {
      // Check if the data is available for m_rinputSef port
      if (m_callback.isDataAvailableForReplyToAsyncPush_m_SefOutput())
      {
        const ::cem::SefOutput_t &rinputSef =
          m_callback.replyToAsyncPush_m_SefOutput();
        return validateStaticObjectProviderInput<config::StaticObjectsSource::SEF_ONLY>(rinputSef);
      }
      return false;
    }

    template<>
    bool AupdfComponentImpl::validateStaticObjectsInput<config::StaticObjectsSource::SGF_ONLY>()
    {
      // Check if the data is available for m_rinputSef port
      if (m_callback.isDataAvailableForReplyToAsyncPush_m_SgfOutput())
      {
        const ::cem::SgfOutput_t &rinputSgf =
          m_callback.replyToAsyncPush_m_SgfOutput();
        return validateStaticObjectProviderInput<config::StaticObjectsSource::SGF_ONLY>(rinputSgf);
      }
      return false;
    }

    template<>
    bool AupdfComponentImpl::validateStaticObjectsInput<config::StaticObjectsSource::SEF_SGF_FUSED>()
    {
      return validateStaticObjectsInput<config::StaticObjectsSource::SEF_ONLY>() && validateStaticObjectsInput<config::StaticObjectsSource::SGF_ONLY>();
    }

    template<>
    void AupdfComponentImpl::fillStaticObjects<config::StaticObjectsSource::SEF_ONLY>(::aupdf::CemOutputs& cemOutputs)
    {
      const ::cem::SefOutput_t &inputSef =
        m_callback.replyToAsyncPush_m_SefOutput();
      m_timestampsOfLastUpdate.sef = inputSef.signalHeader.uiTimeStamp;

      convertSefToAupdfFormat(cemOutputs.static_objects, inputSef);
    }

    template<>
    void AupdfComponentImpl::fillStaticObjects<config::StaticObjectsSource::SGF_ONLY>(::aupdf::CemOutputs& cemOutputs)
    {
      const ::cem::SgfOutput_t &inputSgf =
        m_callback.replyToAsyncPush_m_SgfOutput();
      m_timestampsOfLastUpdate.sgf = inputSgf.signalHeader.uiTimeStamp;

      convertSefToAupdfFormat(cemOutputs.static_objects, inputSgf);
    }

    template<>
    bool AupdfComponentImpl::validateElementForFusion<SefOutput_t>(const cem::StaticElement& staticElement)
    {
      return
        ((staticElement.categoryAssumption == StaticElementCategory::staticElementCategoryUnknown) && !aupdf::config::MERGE_UNKNOWN_USE_SGF) ||
        ((staticElement.categoryAssumption == StaticElementCategory::staticElementCategoryCurbstone) && !aupdf::config::MERGE_CURB_USE_SGF) ||
        ((staticElement.categoryAssumption == StaticElementCategory::staticElementCategoryObstacle) && !aupdf::config::MERGE_STATIC_USE_SGF) ||
        ((staticElement.categoryAssumption == StaticElementCategory::staticElementCategoryTrafficParticipant) && !aupdf::config::MERGE_DYNAMIC_USE_SGF);
    }

    template<>
    bool AupdfComponentImpl::validateElementForFusion<SgfOutput_t>(const cem::StaticElement& staticElement)
    {
      return
        ((staticElement.categoryAssumption == StaticElementCategory::staticElementCategoryUnknown) && aupdf::config::MERGE_UNKNOWN_USE_SGF) ||
        ((staticElement.categoryAssumption == StaticElementCategory::staticElementCategoryCurbstone) && aupdf::config::MERGE_CURB_USE_SGF) ||
        ((staticElement.categoryAssumption == StaticElementCategory::staticElementCategoryObstacle) && aupdf::config::MERGE_STATIC_USE_SGF) ||
        ((staticElement.categoryAssumption == StaticElementCategory::staticElementCategoryTrafficParticipant) && aupdf::config::MERGE_DYNAMIC_USE_SGF);
    }

    template<typename OutputType>
    void AupdfComponentImpl::mergeOutput(cem::SefOutput_t& result, const OutputType& appendingOutput)
    {
      for (size_t i{0}; i < appendingOutput.numberOfStaticElements; ++i)
      {
        const auto& staticElementToAdd{ appendingOutput.staticElements[i] };

        bool maxNumberOfElementsReached{ result.numberOfStaticElements >= ::cem::SymbolicConstants::AGP_MAX_NUM_ELEMENTS };
        if (maxNumberOfElementsReached)
        {
          break;
        }

        bool weWantToMerge = validateElementForFusion<OutputType>(staticElementToAdd);
        if (weWantToMerge)
        {
          auto newStartVertexIndex{0};
          if (result.numberOfStaticElements > 0)
          {
            const auto& resultLastObject{ result.staticElements[result.numberOfStaticElements - 1] };
            newStartVertexIndex = resultLastObject.firstVertexIndex + resultLastObject.numberOfVertices;
          }

          bool maxNumberOfVerticesReached{ newStartVertexIndex + staticElementToAdd.numberOfVertices > ::cem::SymbolicConstants::AGP_MAX_NUM_VERTICES };
          if (maxNumberOfVerticesReached)
          {
            break;
          }

          result.staticElements[result.numberOfStaticElements] = staticElementToAdd;
          result.staticElements[result.numberOfStaticElements].firstVertexIndex = newStartVertexIndex;
          for (size_t vertexIdx{ 0 }; vertexIdx < staticElementToAdd.numberOfVertices; ++vertexIdx)
          {
            result.staticElementVertices[newStartVertexIndex + vertexIdx] = appendingOutput.staticElementVertices[staticElementToAdd.firstVertexIndex + vertexIdx];
          }
          ++result.numberOfStaticElements;
        }
      }
    }

    template<>
    void AupdfComponentImpl::fillStaticObjects<config::StaticObjectsSource::SEF_SGF_FUSED>(::aupdf::CemOutputs& cemOutputs)
    {
      const ::cem::SefOutput_t &sefOutput = m_callback.replyToAsyncPush_m_SefOutput();
      const ::cem::SgfOutput_t &sgfOutput = m_callback.replyToAsyncPush_m_SgfOutput();

      m_timestampsOfLastUpdate.sef = sefOutput.signalHeader.uiTimeStamp;

      if (sefOutput.numberOfStaticElements == 0)
      {
        convertSefToAupdfFormat(cemOutputs.static_objects, sgfOutput);
      }
      else
      {
        cem::SefOutput_t mergedOutput{};
        mergedOutput.signalHeader = sefOutput.signalHeader;

        mergeOutput<SefOutput_t>(mergedOutput, sefOutput);
        mergeOutput<SgfOutput_t>(mergedOutput, sgfOutput);

        convertSefToAupdfFormat(cemOutputs.static_objects, mergedOutput);
      }
    }

    bool AupdfComponentImpl::execute(const uint8 runable) {
      CompOpState opState = ::cem::CompOpState::OPS_DONE;
      bool executionSuccessful = false;
      bool isPfsValid = false;
      bool isEmlValid = false;
      bool isDynValid = false;
      bool dataUnavailable = false;

      bool areStaticObjectsValid{ validateStaticObjectsInput<config::STATIC_OBJECT_SOURCE>() };

      // Check if the data is available for m_rinputPfs port
      if (m_callback.isDataAvailableForReplyToAsyncPush_m_pPfsOutput()) {
        const ::cem::PfsOutput_t &rinputPfs =
            m_callback.replyToAsyncPush_m_pPfsOutput();
        isPfsValid = validatePfsInput(rinputPfs);
      }

      // Check if the data is available for m_rinputEml port
      if (m_callback.isDataAvailableForReplyToAsyncPush_m_egoVehicleKinematicsQueue()) {
        const ::cem::EgoVehicleKinematicsQueue &rinputEml = m_callback.replyToAsyncPush_m_egoVehicleKinematicsQueue();
        isEmlValid = validateEmlInput(rinputEml);
      }

      // Check if the data is available for m_rinputTPF2 port
      if (m_callback.isDataAvailableForReplyToAsyncPush_m_tpObjectList()) {
        const ::cem::TP_t_ObjectList &rinputTPF2 =
            m_callback.replyToAsyncPush_m_tpObjectList();
        isDynValid = validateDynInput(rinputTPF2);
      }

      if (!areStaticObjectsValid && !isPfsValid && !isEmlValid && !isDynValid) {
        dataUnavailable = true;
      }

      if (!dataUnavailable) {
        if (m_validServiceInit == true) {
          ::aupdf::CemOutputs &cemOutputs =
              m_callback.getTriggerAsyncPushBuffer_CEMOutputs();
          if (areStaticObjectsValid) {
            initializeDataStruct(cemOutputs.static_objects);
            fillStaticObjects<config::STATIC_OBJECT_SOURCE>(cemOutputs);
          }

          if (isPfsValid)
          {
            const ::cem::PfsOutput_t &pfsOutput = m_callback.replyToAsyncPush_m_pPfsOutput();
            initializeDataStruct(cemOutputs.parking_delimiters);
            m_timestampsOfLastUpdate.pfs = pfsOutput.signalHeader.uiTimeStamp;

            convertPclToAupdfFormat(cemOutputs.parking_delimiters, pfsOutput);
            convertPsdToAupdfFormat(cemOutputs.parking_slots, pfsOutput);
          }

          const ::cem::EgoVehicleKinematicsQueue &inputEml = m_callback.replyToAsyncPush_m_egoVehicleKinematicsQueue();
          if (isEmlValid) {
            initializeDataStruct(cemOutputs.ego_motion_at_cem_time);
            m_timestampsOfLastUpdate.eml = inputEml.sigHeader.uiTimeStamp;

            convertEmlToAupdfFormat(cemOutputs.ego_motion_at_cem_time, inputEml);
          }

          if (isDynValid) {
            const ::cem::TP_t_ObjectList &inputDynObj =
                m_callback.replyToAsyncPush_m_tpObjectList();
            initializeDataStruct(cemOutputs.dynamic_objects);
            m_timestampsOfLastUpdate.dyn = inputDynObj.sSigHeader.uiTimeStamp;

            convertTPF2ToAupdfFormat(cemOutputs.dynamic_objects, inputDynObj);
          }
          fillCemOutputHeader(cemOutputs, isEmlValid, inputEml);
        } else {
          opState = ::cem::CompOpState::OPS_CANCELED;
        }

        if (areStaticObjectsValid || isPfsValid || isEmlValid || isDynValid)
        {
          m_callback.triggerAsyncPush_CEMOutputs();
        }
        else
        {
          opState = ::cem::CompOpState::OPS_CANCELED;
        }

        if (opState == ::cem::CompOpState::OPS_DONE) {
          executionSuccessful = true;
        }
      }

      return executionSuccessful;
    }

    void AupdfComponentImpl::fillCemOutputHeader(::aupdf::CemOutputs& cemOutputs, bool bEmlValid, const ::cem::EgoVehicleKinematicsQueue &inputEml)
    {
      if (bEmlValid) {
        cemOutputs.header.timestamp = inputEml.sigHeader.uiTimeStamp;
      } else {
        cemOutputs.header.timestamp = cemOutputs.static_objects.header.timestamp;
      }
    }

    template<typename T>
    void AupdfComponentImpl::convertSefToAupdfFormat(::aupdf::SefOutput &outputAupdfFormat, const T &inputStaticObjectProvider)
    {
      outputAupdfFormat.header.timestamp = inputStaticObjectProvider.signalHeader.uiTimeStamp;
      outputAupdfFormat.sef_data_agp.timestamp_us = inputStaticObjectProvider.signalHeader.uiTimeStamp;
      outputAupdfFormat.sef_data_vertices.timestamp_us = inputStaticObjectProvider.signalHeader.uiTimeStamp;
      outputAupdfFormat.cem_sef_signal_state_nu = convertSigState(inputStaticObjectProvider.signalHeader.eSigStatus);

      const uint32 maxNumUsedElem{ CML_Min(inputStaticObjectProvider.numberOfStaticElements, ::aupdf::Constants::SEF_AGP_MAX_ELEMENTS) };
      uint32 elementIdx{};
      while (elementIdx < maxNumUsedElem)
      {
        ::aupdf::SefAGPElement &outputElement = outputAupdfFormat.sef_data_agp.sef_a_element_array_t[elementIdx];
        const cem::StaticElement &inputElement = inputStaticObjectProvider.staticElements[elementIdx];

        const uint16 lastVerticeIdx = inputElement.firstVertexIndex + inputElement.numberOfVertices;
        if (lastVerticeIdx > ::aupdf::Constants::SEF_AGP_MAX_VERTICES)
        {
          break;
        }

        switch (inputElement.drivability)
        {
          case ::cem::StaticElementDrivability::staticElementDrivabilityUnknown:
          {
            outputElement.e_element_semantic = ::cem::e_ElementSemantic_t::OBSTACLE;
            outputElement.e_element_classification = ::cem::e_ElementClassification_t::AGP_ITEM_CLASS_UNKNOWN;
            break;
          }
          case ::cem::StaticElementDrivability::staticElementDrivabilityObstruction:
          {
            outputElement.e_element_semantic = ::cem::e_ElementSemantic_t::OBSTACLE;
            outputElement.e_element_classification = ::cem::e_ElementClassification_t::AGP_ITEM_CLASS_UNKNOWN;
            break;
          }
          case ::cem::StaticElementDrivability::staticElementDrivabilityOverdrivable:
          {
            outputElement.e_element_semantic = ::cem::e_ElementSemantic_t::FREE;
            outputElement.e_element_classification = ::cem::e_ElementClassification_t::AGP_ITEM_CLASS_CURBSTONE;
            break;
          }
          case ::cem::StaticElementDrivability::staticElementDrivabilityBodyTraversable:
          {
            outputElement.e_element_semantic = ::cem::e_ElementSemantic_t::OBSTACLE;
            outputElement.e_element_classification = ::cem::e_ElementClassification_t::AGP_ITEM_CLASS_CURBSTONE;
            break;
          }
          default:
          {
            outputElement.e_element_semantic = ::cem::e_ElementSemantic_t::FREE;
            outputElement.e_element_classification = ::cem::e_ElementClassification_t::AGP_ITEM_CLASS_UNKNOWN;
          }
        }

        outputElement.u_id = inputElement.id;
        outputElement.u_num_vertices = inputElement.numberOfVertices;
        outputElement.s_center.point_3d.f_x_r =
            (inputElement.boundingBox.maxCorner.x +
             inputElement.boundingBox.minCorner.x) /
            2.0F;
        outputElement.s_center.point_3d.f_y_a =
            (inputElement.boundingBox.maxCorner.y +
             inputElement.boundingBox.minCorner.y) /
            2.0F;
        // no inputs
        outputElement.s_center.point_3d.f_z_e = 0.0F;
        outputElement.s_center.point_3d.f_xr_var = 0.0F;
        outputElement.s_center.point_3d.f_xy_ra_cov = 0.0F;
        outputElement.s_center.point_3d.f_xz_re_cov = 0.0F;
        outputElement.s_center.point_3d.f_ya_var = 0.0F;
        outputElement.s_center.point_3d.f_yz_ae_cov = 0.0F;
        outputElement.s_center.point_3d.f_ze_var = 0.0F;

        outputElement.u_vertex_start_index = inputElement.firstVertexIndex;
        for (uint16 vertexIdx = outputElement.u_vertex_start_index; vertexIdx < lastVerticeIdx; ++vertexIdx)
        {
          ++outputAupdfFormat.sef_data_vertices.u_used_vertices;
          ::aupdf::SefAGPVertex &outputVertex = outputAupdfFormat.sef_data_vertices.a_polygon_vertex[vertexIdx];
          const StaticElementVertex &inputVertex = inputStaticObjectProvider.staticElementVertices[vertexIdx];

          outputVertex.point_3d.f_x_r = inputVertex.position.x;
          outputVertex.point_3d.f_y_a = inputVertex.position.y;
          outputVertex.point_3d.f_z_e = inputVertex.position.z;

          outputVertex.point_3d.f_xr_var = inputVertex.positionCovariance.xx;
          outputVertex.point_3d.f_xy_ra_cov = inputVertex.positionCovariance.xy;
          outputVertex.point_3d.f_xz_re_cov = inputVertex.positionCovariance.xz;
          outputVertex.point_3d.f_ya_var = inputVertex.positionCovariance.yy;
          outputVertex.point_3d.f_yz_ae_cov = inputVertex.positionCovariance.yz;
          outputVertex.point_3d.f_ze_var = inputVertex.positionCovariance.zz;
        }
        ++elementIdx;
      }
      outputAupdfFormat.sef_data_agp.u_used_elements = elementIdx;
    }

    void AupdfComponentImpl::convertPclToAupdfFormat(::aupdf::PclOutput &outputAupdfFormat, const cem::PfsOutput_t &inputPfs)
    {
        // PCL: PCL + WS
        outputAupdfFormat.header.timestamp = inputPfs.signalHeader.uiTimeStamp;
        // outputAupdfFormat.header.timestampSource  not used
        // outputAupdfFormat.header.timestampSyncState not used
        outputAupdfFormat.cem_pcl_signal_state_nu =
            convertSigState(inputPfs.signalHeader.eSigStatus);
        const uint32 minWsBuffer = 10U;
        const uint32 numPclDelOut = CML_Min(inputPfs.pclOutput.numberOfDelimiters, ::aupdf::Constants::PFS_PCL_MAX_DELIMITERS - minWsBuffer);
        const uint32 numWsDelOut = CML_Min(inputPfs.wsOutput.numberOfDelimiters, ::aupdf::Constants::PFS_PCL_MAX_DELIMITERS - numPclDelOut);
		outputAupdfFormat.num_pcl_delimiters = 0;
        for (uint32 i = 0U; i < numPclDelOut; i++)
        {
            if (inputPfs.pclOutput.pclDelimiters[i].attributes.relationToPSDslots != relationToPsd_t::ContradictsAssociated)
            { // Filtered lines should not be sent to the output
                convertPclDelimiter(inputPfs.pclOutput.pclDelimiters[i], outputAupdfFormat.cem_pcl_delimiters[outputAupdfFormat.num_pcl_delimiters]);
				outputAupdfFormat.num_pcl_delimiters++;
            }
        }

        for (uint32 j = 0; j < numWsDelOut; j++)
        {
			if (inputPfs.wsOutput.pclDelimiters[j].attributes.relationToPSDslots != relationToPsd_t::ContradictsAssociated) {
				// Filtered lines should not be sent to the output
				convertPclDelimiter(inputPfs.wsOutput.pclDelimiters[j], outputAupdfFormat.cem_pcl_delimiters[outputAupdfFormat.num_pcl_delimiters]);
				outputAupdfFormat.num_pcl_delimiters++;
			}
        }
    }

    void AupdfComponentImpl::convertPclDelimiter(
      const cem::PclDelimiter_t& in,
      ::aupdf::PclDelimiter_t& out)
    {
      out.confidence_percent = in.confidencePercent;
      out.contributing_sensors = in.contributingSensors;
      out.delimiterId = in.delimiterId;
      out.delimiterType.data = static_cast<uint8>(in.delimiterType);
      out.end_point_x_position = in.endPointXPosition;
      out.end_point_y_position = in.endPointYPosition;
      out.is_end_seen = in.isEndSeen;
      out.is_start_seen = in.isStartSeen;
      out.normal_vector.f_xr = in.normalVector.f_Xr;
      out.normal_vector.f_ya = in.normalVector.f_Ya;
      out.start_point_x_position = in.startPointXPosition;
      out.start_point_y_position = in.startPointYPosition;
    }

    void AupdfComponentImpl::convertPsdToAupdfFormat(::aupdf::ParkingSlotDetectionOutput &outputAupdfFormat,
                                                     const cem::PfsOutput_t &inputPfs)
      {
      outputAupdfFormat.header.timestamp = inputPfs.signalHeader.uiTimeStamp;
      // outputAupdfFormat.header.timestampSource  not used
      // outputAupdfFormat.header.timestampSyncState not used
      outputAupdfFormat.cem_signal_state =
          convertSigState(inputPfs.signalHeader.eSigStatus);

      const uint16 numSlotOut = CML_Min(inputPfs.psdOutput.numberOfSlots, ::aupdf::Constants::PFS_PSD_MAX_PARKING_SLOTS);
      outputAupdfFormat.number_of_slots = numSlotOut;

      for (uint16 i = 0U; i < numSlotOut; i++)
      {
        outputAupdfFormat.parking_slots[i].slotId = inputPfs.psdOutput.parkingSlotTracks[i].attributes.trackId;
        for (uint8 corner = 0; corner < 4; ++corner)
        {
          outputAupdfFormat.parking_slots[i].slot_corners[corner].x =
              inputPfs.psdOutput.parkingSlotTracks[i].slot.rectangleGeometry.slotCorners[corner].x;
          outputAupdfFormat.parking_slots[i].slot_corners[corner].y =
              inputPfs.psdOutput.parkingSlotTracks[i].slot.rectangleGeometry.slotCorners[corner].y;
          outputAupdfFormat.parking_slots[i].delimiter_type[corner].data =
              static_cast<uint8>(inputPfs.psdOutput.parkingSlotTracks[i].slot.delimiterType[corner]);
          outputAupdfFormat.parking_slots[i].corner_occlusion_state[corner] =
              inputPfs.psdOutput.parkingSlotTracks[i].slot.cornerOcclusionState[corner];
        }
        outputAupdfFormat.parking_slots[i].existence_probability =
            inputPfs.psdOutput.parkingSlotTracks[i].slot.existenceProbability;
        outputAupdfFormat.parking_slots[i].parking_scenario_confidence.angled =
            inputPfs.psdOutput.parkingSlotTracks[i].slot.parkingScenarioConfidence.angled;
        outputAupdfFormat.parking_slots[i].parking_scenario_confidence.parallel =
            inputPfs.psdOutput.parkingSlotTracks[i].slot.parkingScenarioConfidence.parallel;
        outputAupdfFormat.parking_slots[i].parking_scenario_confidence.perpendicular =
            inputPfs.psdOutput.parkingSlotTracks[i].slot.parkingScenarioConfidence.perpendicular;
      }
    }

    void AupdfComponentImpl::convertEmlToAupdfFormat(
        ::aupdf::EgoMotionAtCemOutput &outputAupdfFormat,
        const cem::EgoVehicleKinematicsQueue &inputEml) {
      // See cem_eml_interpolator.cpp in cemlite
      EgoVehicleKinematicsQueue emlOutput(inputEml);
      cem::eml::EmlAccess emlAccess(emlOutput);
      cem::EgoVehicleKinematics egoVehKin = emlAccess.getHeadElement();
      EgoVehicleKinematics eml;

      outputAupdfFormat.vehicle_pose_est_at_cem_time.timestamp_us = egoVehKin.sigHeader.uiTimeStamp;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.signal_state_nu = convertSigState(egoVehKin.sigHeader.eSigStatus);

      outputAupdfFormat.vehicle_pose_est_at_cem_time.longi_velocity_mps = egoVehKin.egoMotion.velocityTrans.x.mu;
      // not avialable anymore odoOutput.vehVelocityCurSigma_mps  = eml.egoMotion.velocityTrans.x.sig;

      outputAupdfFormat.vehicle_pose_est_at_cem_time.x_position_m = egoVehKin.localization.localPose.position.x.mu;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.x_position_var_m2 = egoVehKin.localization.localPose.position.x.sig*egoVehKin.localization.localPose.position.x.sig;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.y_position_m = egoVehKin.localization.localPose.position.y.mu;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.y_position_var_m2 = egoVehKin.localization.localPose.position.y.sig*egoVehKin.localization.localPose.position.y.sig;
      // not avialable anymore odoOutput.zEgoRACur_m              = eml.localization.localPose.position.z.mu;
      // not avialable anymore odoOutput.zEgoRACurSigma_m         = eml.localization.localPose.position.z.sig;

      outputAupdfFormat.vehicle_pose_est_at_cem_time.x_velocity_mps = egoVehKin.egoMotion.velocityTrans.x.mu;
      // not avialable anymore odoOutput.xVelEgoRaCurSigma_mps    = eml.egoMotion.velocityTrans.x.sig;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.y_velocity_mps = egoVehKin.egoMotion.velocityTrans.y.mu;
      // not avialable anymore odoOutput.yVelEgoRaCurSigma_mps    = eml.egoMotion.velocityTrans.y.sig;
      // not avialable anymore odoOutput.zVelEgoRaCur_mps         = eml.egoMotion.velocityTrans.z.mu;
      // not avialable anymore odoOutput.zVelEgoRaCurSigma_mps    = eml.egoMotion.velocityTrans.z.sig;

      outputAupdfFormat.vehicle_pose_est_at_cem_time.yaw_angle_rad = egoVehKin.localization.localPose.rotationParameters.yaw.mu;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.yaw_angle_var_rad2 = egoVehKin.localization.localPose.rotationParameters.yaw.sig*egoVehKin.localization.localPose.rotationParameters.yaw.sig;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.roll_angle_rad = egoVehKin.localization.localPose.rotationParameters.roll.mu;
      // not avialable anymore odoOutput.rollAngEgoCurSigma_rad   = eml.localization.localPose.rotationParameters.roll.sig;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.pitch_angle_rad = egoVehKin.localization.localPose.rotationParameters.pitch.mu;
      // not avialable anymore odoOutput.pitchAngEgoCurSigma_rad  = eml.localization.localPose.rotationParameters.pitch.sig;

      outputAupdfFormat.vehicle_pose_est_at_cem_time.yaw_rate_radps = egoVehKin.egoMotion.velocityAng.z.mu;
      // not avialable anymore odoOutput.yawRateEgoCurSigma_rad   = eml.egoMotion.velocityAng.z.sig;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.roll_rate_radps = egoVehKin.egoMotion.velocityAng.x.mu;
      // not avialable anymore odoOutput.rollRateEgoCurSigma_rad  = eml.egoMotion.velocityAng.x.sig;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.pitch_rate_radps = egoVehKin.egoMotion.velocityAng.y.mu;
      // not avialable anymore odoOutput.pitchRateEgoCurSigma_rad = eml.egoMotion.velocityAng.y.sig;

      outputAupdfFormat.vehicle_pose_est_at_cem_time.steer_ang_front_axle_rad = 0.0F; //???? const 0 ???
      outputAupdfFormat.vehicle_pose_est_at_cem_time.longi_acceleration_mps2 = 0.0F;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.xy_position_var_m2 = 0.0F;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.yx_position_var_m2 = 0.0F;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.steer_ang_rear_axle_rad = 0.0F;
      outputAupdfFormat.vehicle_pose_est_at_cem_time.driven_distance_m = 0.0F;

      if (egoVehKin.egoMotion.motionState.longMotStateOverall == cem::Cem200LongMotStates::LONG_MOT_STATE_STDST)
      {
        outputAupdfFormat.vehicle_pose_est_at_cem_time.motion_status_nu = ::lsm_vedodo::MotionState::ODO_STANDSTILL;
        outputAupdfFormat.vehicle_pose_est_at_cem_time.driving_direction_nu = ::lsm_vedodo::Direction::DIRECTION_UNDEFINED;
      }
      else
      {
        outputAupdfFormat.vehicle_pose_est_at_cem_time.motion_status_nu = ::lsm_vedodo::MotionState::ODO_NO_STANDSTILL;

        if (egoVehKin.egoMotion.motionState.longMotStateOverall == cem::Cem200LongMotStates::LONG_MOT_STATE_MOVE_FWD)
        {
          outputAupdfFormat.vehicle_pose_est_at_cem_time.driving_direction_nu = ::lsm_vedodo::Direction::DIRECTION_FORWARD;
        }
        else if (egoVehKin.egoMotion.motionState.longMotStateOverall == cem::Cem200LongMotStates::LONG_MOT_STATE_MOVE_RWD)
        {
          outputAupdfFormat.vehicle_pose_est_at_cem_time.driving_direction_nu = ::lsm_vedodo::Direction::DIRECTION_REVERSE;
        }
        else
        {
          outputAupdfFormat.vehicle_pose_est_at_cem_time.driving_direction_nu = ::lsm_vedodo::Direction::DIRECTION_UNDEFINED;
        }
      }
    }

    void AupdfComponentImpl::convertTPF2ToAupdfFormat(::aupdf::DynamicEnvironment& dynObjOutput,
      const ::cem::TP_t_ObjectList& dynObjInput)
    {
      dynObjOutput.header.timestamp = dynObjInput.sSigHeader.uiTimeStamp;

      const uint16 numOfObjects = CML_Min(dynObjInput.HeaderObjList.iNumOfUsedObjects, ::aupdf::Constants::TPF_MAX_DYN_OBJECTS);
      dynObjOutput.numberOfObjects = numOfObjects;

      for (sint32 i = 0; i < numOfObjects; i++)
      {
        cem::ObjNumber_t index = dynObjInput.HeaderObjList.iSortedObjectList[i];
        Aupdf_TpfObjConverter::convertCartesianObject(dynObjInput.aObject[index], dynObjOutput.objects[i]);
      }
    }

    ::com::ComSignalState_t AupdfComponentImpl::convertSigState(const cem::AlgoSignalState_t& cemSigState)
    {
      ::com::ComSignalState_t convertedSigState;
      if (cemSigState == cem::AlgoSignalState_t::AL_SIG_STATE_INIT)
      {
        convertedSigState = ::com::ComSignalState_t::COMSIGSTATE_INIT;
      }
      else if (cemSigState == cem::AlgoSignalState_t::AL_SIG_STATE_OK)
      {
        // no differentiation to COMSIGSTATE_TIMEOUT
        convertedSigState = ::com::ComSignalState_t::COMSIGSTATE_VALID;
      }
      else
      {
        // no differentiation to COMSIGSTATE_TIMEOUT
        convertedSigState = ::com::ComSignalState_t::COMSIGSTATE_INVALID;
      }

      return convertedSigState;
    }

    bool AupdfComponentImpl::suspend(void)
    {
      return true;
    }

    bool AupdfComponentImpl::resume(void)
    {
      return true;
    }

    bool AupdfComponentImpl::shutdown(void)
    {
      return true;
    }

    bool AupdfComponentImpl::setExecutionMode(const ::eco::ComponentExecutionMode executionMode)
    {
      m_executionMode = executionMode;
      return true;
    }
  }
}
