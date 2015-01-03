//=============================================================================
/** @file parapet_config.h
 *
 * @brief
 *	Macros used to configure support for optional features in the parapet
 *	library
 *
 * @author Dave Billin
 */
//=============================================================================

#ifndef PARAPET_CONFIG_H_
#define PARAPET_CONFIG_H_


//=============================================================================
/// The maximum total size of a parapet packet (header + payload) in Bytes
enum { PARAPET_MAX_BYTES_PER_PACKET = 1400UL };


//=============================================================================
/** @DEF PARAPET_ENABLE_HEADER_CHECKSUM
 * @brief
 *   Set to a non-zero value to enable creation and verification of the
 *   CHECKSUM field in parapet packet headers.
 *
 * @details
 *   If PARAPET_ENABLE_HEADER_CHECKSUMS is defined as a non-zero value, support
 *   for the CHECKSUM field in parapet functions will be enabled.  That is,
 *   functions that initialize packets will automatically populate the CHECKSUM
 *   field in packet headers, and functions that verify packets will validate
 *   received header checksum values.
 *
 *   If PARAPET_ENABLE_HEADER_CHECKSUMS is defined as zero, support for the
 *   CHECKSUM field in parapet functions will be disabled.  Functions that
 *   initialize packets will always set the CHECKSUM field to zero, and
 *   functions that verify packets will disregard received checksums
 */
#define PARAPET_ENABLE_HEADER_CHECKSUM



//=============================================================================
/** @def PARAPET_ENABLE_BROADCAST_ID
 * @brief
 *   Define this symbol to enable support for a 'broadcast' device ID that
 *   can be used to address all devices on a shared bus
 *
 * @details
 *   If PARAPET_ENABLE_BROADCAST_ID is defined, support will be enabled for a
 *   single device ID on the network (designated by the value of the macro
 *   PARAPET_BROADCAST_DEVICE_ID) to be treated as a 'broadcast' ID that
 *   addresses all devices.  Note that logic governing whether or not to
 *   respond to a broadcast packet - which could lead to a data collision in
 *   some bus topologies - is left to the implementor.
 *
 *   If PARAPET_ENABLE_BROADCAST_ID is not defined, no broadcast ID will be
 *   assigned, and support for broadcast packets will be disabled in parapet
 *   functions.
 */
#define PARAPET_ENABLE_BROADCAST_ID


//=============================================================================
/** @def PARAPET_BROADCAST_DEVICE_ID
 * @brief
 *   Device ID designated as a 'broadcast' identifier that addresses all
 *   devices when the PARAPET_ENABLE_BROADCAST_ID feature is enabled.
 *
 * @details
 *   If PARAPET_ENABLE_BROADCAST_ID is defined, then the symbol
 *   PARAPET_BROADCAST_DEVICE_ID is used to desginate a device ID that will be
 *   regarded as a 'broadcast' address, specifying all devices.  Otherwise,
 *   PARAPET_BROADCAST_DEVICE_ID is not defined.
 */
#ifdef PARAPET_ENABLE_BROADCAST_ID
#define PARAPET_BROADCAST_DEVICE_ID 0x0f
#endif



/** @def PARAPET_ENABLE_ASSERTS
 * @brief
 *   Define this symbol to enable assertion statements in parapet support
 *   functions when debugging
 */
#define PARAPET_ENABLE_ASSERTS 1

#endif /* PARAPET_CONFIG_H_ */
