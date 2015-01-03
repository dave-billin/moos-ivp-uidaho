//=============================================================================
/*  Copyright (C) 2013  Dave Billin

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//-----------------------------------------------------------------------------
/** @file parapet_descriptor.h
 *
 * @brief
 *  Declaration of a parapet descriptor data structure along with associated
 *  constants, macros, and functions
 *
 * @author Dave Billin <david.billin@vandals.uidaho.edu>
 */
//=============================================================================

#ifndef PARAPET_DESCRIPTOR_H_
#define PARAPET_DESCRIPTOR_H_

#include <stdint.h>

#ifdef __cplusplus
namespace parapet
{
#endif

//=============================================================================
/// Descriptor that appears after a packet header in requests
typedef union
{
    uint32_t u32;
    uint16_t u16[ sizeof(uint32_t) / sizeof(uint16_t) ];
} parapet_descriptor_t;


//=============================================================================
/// Bit offsets of fields in a Descriptor
enum
{
    DESCRIPTOR_OFFSET_ID = 0,           ///< Bit offset of ID field
    DESCRIPTOR_OFFSET_USER_BITS = 16,   ///< Bit offset of user bits
};


//=============================================================================
/// Masks corresponding to Descriptor fields
#define DESCRIPTOR_MASK_ID          0x0000ffff
#define DESCRIPTOR_MASK_USER_BITS   0xffff0000



//=============================================================================
/** @addtogroup parapet_descriptor_read_macros Macros to read descriptor fields
 * @brief
 *   A selection of macros that may be used to extract the value of fields in
 *   a parapet Descriptor
 *
 * @{
 */
//=============================================================================

//--------------------------------------------------------------------------
/** @def PARAPET_DESCRIPTOR_GET_ID
 * @brief Extracts the value of the ID field in a parapet_descriptor_t
 * @param _descriptor_   parapet_descriptor_t struct to extract from
 */
#define PARAPET_DESCRIPTOR_GET_ID( _descriptor_ ) \
    ( (uint16_t)( (_descriptor_).u32 & DESCRIPTOR_MASK_ID ) )

//--------------------------------------------------------------------------
/** @def PARAPET_DESCRIPTOR_GET_USER_BITS
 * @brief Extracts the value of the USER_BITS field in a parapet_descriptor_t
 * @param _descriptor_   parapet_descriptor_t struct to extract from
 */
#define PARAPET_DESCRIPTOR_GET_USER_BITS( _descriptor_ ) \
    ( (uint16_t)( ( (_descriptor_).u32 & DESCRIPTOR_MASK_USER_BITS ) \
                       >> DESCRIPTOR_OFFSET_USER_BITS ) )


///< @{
//=============================================================================



//=============================================================================
/** @addtogroup parapet_descriptor_read_macros Macros to read descriptor fields
 * @brief
 *   A selection of macros that may be used to extract the value of fields in
 *   a parapet Descriptor
 *
 * @{
 */
//=============================================================================

//--------------------------------------------------------------------------
/** @def PARAPET_DESCRIPTOR_SET_ID
 * @brief Sets the ID field in a parapet_descriptor_t struct
 * @param _descriptor_  parapet_descriptor_t struct to operate on
 * @param _id_          ID field value
 */
#define PARAPET_DESCRIPTOR_SET_ID( _descriptor_, _id_ ) \
        ( (_descriptor_).u32 = \
        ( (_descriptor_).u32 & ~DESCRIPTOR_MASK_ID) | ( (_id_) & 0xffff ) )


//--------------------------------------------------------------------------
/** @def PARAPET_DESCRIPTOR_SET_ID
 * @brief Sets the USER_BITS field in a parapet_descriptor_t struct
 * @param _descriptor_  parapet_descriptor_t struct to operate on
 * @param _bits_        USER_BITS field value
 */
#define PARAPET_DESCRIPTOR_SET_USER_BITS( _descriptor_, _bits_ ) \
        ( (_descriptor_).u32 = \
                ( (_descriptor_).u32 & ~DESCRIPTOR_MASK_USER_BITS) | \
                ( ( (_bits_) & 0xffff ) ) << DESCRIPTOR_OFFSET_USER_BITS )

///< @{
//=============================================================================



//--------------------------------------------------------------------------
/** @def InitDescriptor
 * @brief
 *   Initializes the fields of a parapet_descriptor_t with specified values
 *
 * @param _descriptor_   parapet_descriptor_t struct to operate on
 * @param _id_           Value to assign to the descriptor ID field
 * @param _user_bits_    Value to assign to the descriptor USER_BITS field
 */
#define InitDescriptor( _descriptor_, _id_, _user_bits_ ) \
    (_descriptor_).u32 = \
        ( (_id_) & 0xffff ) | \
        (( (_user_bits_) & 0xffff ) << DESCRIPTOR_OFFSET_USER_BITS )


#ifdef __cplusplus
}   // END namespace parapet
#endif

#endif /* PARAPET_DESCRIPTOR_H_ */
