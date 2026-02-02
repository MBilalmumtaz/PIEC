//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#ifndef ZEN_PROPERTIES_GNSSSENSORPROPERTIESV1_H_
#define ZEN_PROPERTIES_GNSSSENSORPROPERTIESV1_H_

#include <array>
#include <cstring>
#include <utility>

#include "InternalTypes.h"
#include "ZenTypes.h"

namespace zen
{
    namespace gnss::v1
    {
        constexpr EDevicePropertyV1 mapCommand(ZenProperty_t command)
        {
            switch (command)
            {
            default:
                return EDevicePropertyV1::Ack;
            }
        }

        constexpr EDevicePropertyV1 map(ZenProperty_t property, bool isGetter)
        {
            const auto set_or = [isGetter](EDevicePropertyV1 prop) {
                return isGetter ? EDevicePropertyV1::Ack : prop;
            };

            switch (property)
            {
            case EZenGnssProperty::ZenGnssProperty_RtkCorrectionMessage:
                return set_or(EDevicePropertyV1::SetRtkCorrection);

            default:
                return EDevicePropertyV1::Ack;
            }
        }
    }
}

#endif
