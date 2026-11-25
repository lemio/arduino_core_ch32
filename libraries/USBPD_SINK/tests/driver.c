#include <assert.h>
#include <stdint.h>

#define __attribute__(x)
#include "../src/usbpd_sink.c"
#undef __attribute__

static void set_pdo(unsigned index, uint32_t pdo)
{
    for (unsigned byte = 0; byte < 4; ++byte)
        usbpdRxBuffer[2 + index * 4 + byte] = pdo >> (8 * byte);
}

static void receive_capabilities(unsigned revision, unsigned count)
{
    USBPD_MessageHeader_t header = {0};
    header.MessageHeader.MessageType = USBPD_DATA_MSG_SRC_CAP;
    header.MessageHeader.NumberOfDataObjects = count;
    header.MessageHeader.SpecificationRevision = revision;
    set_pdo(0, (100u << 10) | 300u);
    set_pdo(1, (3u << 30) | (200u << 17) | (33u << 8) | 60u);
    usbpd_sink_protocol_analysis(&header, &pdControl_g);
    pdControl_g.cc_SinkGoodCRCOver = 1;
    usbpd_sink_process();
}

static USBPD_SINKRDO_t sent_request(void)
{
    USBPD_SINKRDO_t rdo;
    usbpd_sink_process();
    rdo.d32 = (uint32_t)usbpdTxBuffer[2] | ((uint32_t)usbpdTxBuffer[3] << 8) |
              ((uint32_t)usbpdTxBuffer[4] << 16) | ((uint32_t)usbpdTxBuffer[5] << 24);
    return rdo;
}

static void finish_fallback_contract(USBPD_Status_t expected)
{
    pdControl_g.cc_State = CC_PS_RDY;
    pdControl_g.cc_SinkGoodCRCOver = 1;
    usbpd_sink_process();
    assert(pdControl_g.cc_State == CC_ACTIVE);
    assert(usbpd_api_get_status() == expected);
    assert(!usbpd_api_is_ready());
    assert(usbpd_api_get_voltage() == 0);
}

static void ready_contract(void)
{
    pdControl_g.cc_State = CC_PS_RDY;
    pdControl_g.cc_SinkGoodCRCOver = 1;
    usbpd_sink_process();
    assert(usbpd_api_is_ready());
}

int main(void)
{
    usbpd_api_begin();
    usbpd_sink_reset();
    assert(usbpd_api_request_voltage(7, 1000));
    receive_capabilities(DEF_PD_REVISION_30, 2);
    assert(pdControl_g.cc_PD_Version == DEF_PD_REVISION_30);
    assert(pdControl_g.cc_State == CC_SEND_REQUEST);
    assert(pdControl_g.cc_SetPDONum == 2);
    usbpd_sink_process();
    USBPD_MessageHeader_t sent;
    sent.d16 = usbpdTxBuffer[0] | ((uint16_t)usbpdTxBuffer[1] << 8);
    assert(sent.MessageHeader.SpecificationRevision == DEF_PD_REVISION_30);
    ready_contract();
    assert(usbpd_api_get_voltage() == 7000);
    for (unsigned i = 0; i < USBPD_PPS_REFRESH_TICKS - 1; ++i)
        usbpd_sink_process();
    assert(usbpd_api_is_ready());
    usbpd_sink_process();
    assert(pdControl_g.cc_State == CC_SEND_REQUEST);
    /* The PPS keep-alive does not drop the established contract. */
    assert(usbpd_api_is_ready());
    usbpd_sink_process();
    USBPD_SINKRDO_t rdo;
    rdo.d32 = (uint32_t)usbpdTxBuffer[2] | ((uint32_t)usbpdTxBuffer[3] << 8) |
              ((uint32_t)usbpdTxBuffer[4] << 16) | ((uint32_t)usbpdTxBuffer[5] << 24);
    assert(rdo.SinkPPSRDO.ObjectPosition == 2);
    assert(rdo.SinkPPSRDO.OutputVoltagein20mVunits == 350);
    assert(rdo.SinkPPSRDO.OperatingCurrentIN50mAuints == 20);
    ready_contract();
    assert(ppsRefreshWait == 0);

    receive_capabilities(DEF_PD_REVISION_30, 2);
    assert(pdControl_g.cc_State == CC_SEND_REQUEST);
    ready_contract();
    assert(usbpd_api_get_voltage() == 7000);

    pdControl_g.cc_State = CC_WAIT_ACCEPT;
    USBPD_MessageHeader_t reject = {0};
    reject.MessageHeader.MessageType = USBPD_CONTROL_MSG_REJECT;
    usbpd_sink_protocol_analysis(&reject, &pdControl_g);
    assert(usbpd_api_get_status() == USBPD_STATUS_REJECTED);
    assert(!usbpd_api_is_ready());

    usbpd_sink_end();
    assert(usbpd_api_get_status() == USBPD_STATUS_IDLE);
    assert(usbpd_api_get_capability_count() == 0);
    assert(ppsRefreshWait == 0);

    usbpd_api_begin();
    usbpd_sink_reset();
    assert(usbpd_api_request_voltage(7, 1000));
    receive_capabilities(DEF_PD_REVISION_20, 2);
    assert(usbpd_api_get_status() == USBPD_STATUS_VOLTAGE_UNAVAILABLE);

    /* An unavailable voltage still gets a Request: 5 V, flagged as a mismatch. */
    assert(pdControl_g.cc_State == CC_SEND_REQUEST);
    USBPD_SINKRDO_t fallback = sent_request();
    assert(fallback.SinkFixedVariableRDO.ObjectPosition == 1);
    assert(fallback.SinkFixedVariableRDO.CapabilityMismatch == 1);
    assert(fallback.SinkFixedVariableRDO.OperatingCurrentIn10mAunits == 100);
    finish_fallback_contract(USBPD_STATUS_VOLTAGE_UNAVAILABLE);

    receive_capabilities(DEF_PD_REVISION_30, 1);
    assert(usbpd_api_get_status() == USBPD_STATUS_VOLTAGE_UNAVAILABLE);
    assert(sent_request().SinkFixedVariableRDO.ObjectPosition == 1);
    finish_fallback_contract(USBPD_STATUS_VOLTAGE_UNAVAILABLE);
    receive_capabilities(DEF_PD_REVISION_30, 2);
    assert(pdControl_g.cc_State == CC_SEND_REQUEST);
    assert(pdControl_g.cc_SetPDONum == 2);
    ready_contract();

    assert(usbpd_api_request_voltage(5, 1000));
    assert(pdControl_g.cc_SetPDONum == 1);
    ready_contract();
    for (unsigned i = 0; i < USBPD_PPS_REFRESH_TICKS * 2; ++i)
        usbpd_sink_process();
    assert(usbpd_api_is_ready());
    assert(pdControl_g.cc_State == CC_ACTIVE);

    fixedSourceCap[1].Voltage = 9000;
    fixedSourceCap[1].Current = 2000;
    sourceCapabilities[1].minVoltage = 9000;
    sourceCapabilities[1].maxVoltage = 9000;
    sourceCapabilities[1].current = 2000;
    sourceCapabilities[1].programmable = false;
    assert(usbpd_sink_set_request_fixed_voltage(REQUEST_9v));
    assert(pdControl_g.cc_SetPDONum == 2);
    assert(requestedCapability == 1);
    assert(requestedVoltage == 9000);

    /* Too much current: same fallback, with the current limited to the offer. */
    usbpd_sink_end();
    usbpd_api_begin();
    usbpd_sink_reset();
    assert(usbpd_api_request_voltage(5, 5000));
    receive_capabilities(DEF_PD_REVISION_30, 1);
    fallback = sent_request();
    assert(fallback.SinkFixedVariableRDO.CapabilityMismatch == 1);
    assert(fallback.SinkFixedVariableRDO.OperatingCurrentIn10mAunits == 300);
    finish_fallback_contract(USBPD_STATUS_CURRENT_UNAVAILABLE);

    /* A chosen PPS voltage, rounded down to 20 mV. */
    usbpd_sink_end();
    usbpd_api_begin();
    usbpd_sink_reset();
    assert(usbpd_api_request_programmable(1, 9010, 1000));
    receive_capabilities(DEF_PD_REVISION_30, 2);
    assert(sent_request().SinkPPSRDO.OutputVoltagein20mVunits == 450);
    ready_contract();
    assert(usbpd_api_get_voltage() == 9000);

    /* Outside the option's range: unavailable, and the 5 V fallback. */
    usbpd_sink_end();
    usbpd_api_begin();
    usbpd_sink_reset();
    assert(usbpd_api_request_programmable(1, 21000, 1000));
    receive_capabilities(DEF_PD_REVISION_30, 2);
    assert(sent_request().SinkFixedVariableRDO.ObjectPosition == 1);
    finish_fallback_contract(USBPD_STATUS_VOLTAGE_UNAVAILABLE);

    /* begin() without a request (listing capabilities) answers the source too. */
    usbpd_sink_end();
    usbpd_api_begin();
    usbpd_sink_reset();
    receive_capabilities(DEF_PD_REVISION_30, 2);
    assert(usbpd_api_has_capabilities());
    fallback = sent_request();
    assert(fallback.SinkFixedVariableRDO.ObjectPosition == 1);
    assert(fallback.SinkFixedVariableRDO.CapabilityMismatch == 0);
    assert(fallback.SinkFixedVariableRDO.OperatingCurrentIn10mAunits == 300);
    finish_fallback_contract(USBPD_STATUS_IDLE);

    /* A request that can be met afterwards replaces the fallback. */
    assert(usbpd_api_request_voltage(5, 1000));
    fallback = sent_request();
    assert(fallback.SinkFixedVariableRDO.CapabilityMismatch == 0);
    ready_contract();
    assert(usbpd_api_get_voltage() == 5000);
    return 0;
}
