//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//
#include "Modbus.h"

#include <stdexcept>

namespace
{

    uint16_t lrcLp(uint8_t address, uint16_t function, const std::byte* data, uint16_t length) noexcept
    {
        // TODO:
        // LP Sensor firmware computes the additions for
        // the checksum on a byte-level and not using address
        // and function 2-byte integers, check this is working
        // here
        uint16_t total = address;
        total += function & 0xff;
        total += (function >> 8) & 0xff;
        total += length;
        for (auto i = 0; i < length; ++i) {
            total += std::to_integer<uint8_t>(data[i]);
        }

        return total;
    }

    uint16_t combine(uint8_t least, uint8_t most) noexcept
    {
        return (uint16_t(most) << 8) | least;
    }
}

namespace zen::modbus
{
    void IFrameParser::reset()
    {
        m_frame.data.clear();
    }

    std::unique_ptr<IFrameFactory> make_factory(ModbusFormat format) noexcept
    {
        switch (format)
        {
        case ModbusFormat::LP:
            return std::make_unique<LpFrameFactory>();

        default:
            return {};
        }
    }

    std::unique_ptr<IFrameParser> make_parser(ModbusFormat format) noexcept
    {
        switch (format)
        {
        case ModbusFormat::LP:
            return std::make_unique<LpFrameParser>();

        default:
            return {};
        }
    }


    std::vector<std::byte> LpFrameFactory::makeFrame(uint8_t address, uint16_t function, const std::byte* data, uint16_t length) const
    {
        constexpr uint16_t WRAPPER_SIZE = 9; // 1 (start) + 2 (address) + 2 (function) + 2 (LRC) + 2 (end)
        std::vector<std::byte> frame(WRAPPER_SIZE + 2 + length);

        frame[0] = std::byte(0x3a);
        frame[1] = std::byte(address);
        frame[2] = std::byte(0);
        frame[3] = std::byte(function & 0xff);
        frame[4] = std::byte((function >> 8) & 0xff);
        frame[5] = std::byte(length);
        frame[6] = std::byte(0);
        if (length > 0) {
            std::copy(data, data + length, &frame[7]);
        }

        const uint16_t checksum = lrcLp(address, function, data, length);
        frame[7 + length] = std::byte(checksum & 0xff);
        frame[8 + length] = std::byte((checksum >> 8) & 0xff);
        frame[9 + length] = std::byte(0x0d);
        frame[10 + length] = std::byte(0x0a);

        return frame;
    }

    LpFrameParser::LpFrameParser()
        : m_state(LpFrameParseState::Start)
    {}

    void LpFrameParser::reset()
    {
        IFrameParser::reset();
        m_state = LpFrameParseState::Start;
    }

    FrameParseError LpFrameParser::parse(gsl::span<const std::byte>& data)
    {
        while (!data.empty())
        {
            switch (m_state)
            {
            case LpFrameParseState::Start:
                if (data[0] != std::byte(0x3a))
                    return FrameParseError_ExpectedStart;

                m_state = LpFrameParseState::Address1;
                break;

            case LpFrameParseState::Address1:
                m_buffer = data[0];
                m_state = LpFrameParseState::Address2;
                break;

            case LpFrameParseState::Address2:
                m_frame.address = std::to_integer<uint8_t>(m_buffer);
                m_state = LpFrameParseState::Function1;
                break;

            case LpFrameParseState::Function1:
                m_buffer = data[0];
                m_state = LpFrameParseState::Function2;
                break;

            case LpFrameParseState::Function2:
                m_frame.function = combine(std::to_integer<uint8_t>(m_buffer), std::to_integer<uint8_t>(data[0]));
                m_state = LpFrameParseState::Length1;
                break;

            case LpFrameParseState::Length1:
                m_buffer = data[0];
                m_state = LpFrameParseState::Length2;
                break;

            case LpFrameParseState::Length2:
                m_length = combine(std::to_integer<uint8_t>(m_buffer), std::to_integer<uint8_t>(data[0]));
                m_frame.data.reserve(m_length);
                m_state = m_length != 0 ? LpFrameParseState::Data : LpFrameParseState::Check1;
                break;

            case LpFrameParseState::Data:
                m_frame.data.emplace_back(data[0]);
                m_state = m_frame.data.size() == m_length ? LpFrameParseState::Check1 : LpFrameParseState::Data;
                break;

            case LpFrameParseState::Check1:
                m_buffer = data[0];
                m_state = LpFrameParseState::Check2;
                break;

            case LpFrameParseState::Check2:
                if (combine(std::to_integer<uint8_t>(m_buffer), std::to_integer<uint8_t>(data[0])) != lrcLp(m_frame.address, m_frame.function, m_frame.data.data(), m_length))
                    return FrameParseError_ChecksumInvalid;

                m_state = LpFrameParseState::End1;
                break;

            case LpFrameParseState::End1:
                if (data[0] != std::byte(0x0d))
                    return FrameParseError_ExpectedEnd;

                m_state = LpFrameParseState::End2;
                break;

            case LpFrameParseState::End2:
                if (data[0] != std::byte(0x0a))
                    return FrameParseError_ExpectedEnd;

                m_state = LpFrameParseState::Finished;
                data = data.subspan(1);
                return FrameParseError_None;

            case LpFrameParseState::Finished:
                return FrameParseError_Finished;

            default:
                return FrameParseError_UnexpectedCharacter;
            }

            data = data.subspan(1);
        }

        return FrameParseError_None;
    }

}
