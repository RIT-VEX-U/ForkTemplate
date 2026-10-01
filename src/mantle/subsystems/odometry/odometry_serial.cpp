#include "mantle/subsystems/odometry/odometry_serial.h"
#include "core/utils/time.h"
#include <cassert>
#include <cstring>
#include <cstdio>

OdometrySerial::OdometrySerial(
  mantle::SerialPort &serial, bool is_async, bool calc_vel_acc_on_brain,
  Pose2d initial_pose, Pose2d sensor_offset
)
    : OdometryBase(is_async), serial(serial), calc_vel_acc_on_brain(calc_vel_acc_on_brain),
      pose(), pose_offset() {
    send_config(initial_pose, sensor_offset, calc_vel_acc_on_brain);
}

void OdometrySerial::send_config(
  const Pose2d &initial_pose, const Pose2d &sensor_offset, const bool &calc_vel_acc_on_brain
) {
    uint8_t raw[6 * sizeof(float) + sizeof(calc_vel_acc_on_brain)];
    uint8_t cobs_encoded[sizeof(raw) + 1];

    float initialx = (float)initial_pose.x(units::in);
    float initialy = (float)initial_pose.y(units::in);
    float initialrot = (float)initial_pose.rotation().degrees();

    float offsetx = (float)sensor_offset.x(units::in);
    float offsety = (float)sensor_offset.y(units::in);
    float offsetrot = (float)sensor_offset.rotation().degrees();

    std::memcpy(&raw[0], &initialx, sizeof(float));
    std::memcpy(&raw[4], &initialy, sizeof(float));
    std::memcpy(&raw[8], &initialrot, sizeof(float));
    std::memcpy(&raw[12], &offsetx, sizeof(float));
    std::memcpy(&raw[16], &offsety, sizeof(float));
    std::memcpy(&raw[20], &offsetrot, sizeof(float));
    std::memcpy(&raw[24], &calc_vel_acc_on_brain, sizeof(bool));

    cobs_encode(raw, sizeof(raw), cobs_encoded);

    serial.write(cobs_encoded, sizeof(cobs_encoded));
}

int OdometrySerial::receive_cobs_packet(uint8_t *buffer, size_t buffer_size) {
    size_t index = 0;

    while (true) {
        if (serial.available() > 0) {
            uint8_t character = (uint8_t)serial.read_char();

            // if delimiter
            if (character == 0x00) {
                return (int)index; // return packet length
            }

            // store character in buffer
            if (index < buffer_size) {
                buffer[index++] = character;
            } else {
                printf("bufferoverflow\n");
                return -1;
            }
        }
        core::delay_ms(1);
    }
}

Pose2d OdometrySerial::update() {
    uint8_t cobs_encoded_size = 29;
    uint8_t packet_size = 28;

    uint8_t cobs_encoded[29];
    uint8_t decoded_packet[28];

    int packet_length = receive_cobs_packet(cobs_encoded, cobs_encoded_size);
    Pose2d updated_pose;

    if (packet_length == cobs_encoded_size) {
        if (cobs_decode(cobs_encoded, packet_length, decoded_packet) == packet_size) {
            float *floats = (float *)decoded_packet;

            updated_pose = Pose2d(units::Length(floats[0], units::in), units::Length(floats[1], units::in),
                                  Rotation2d(units::Angle(floats[2], units::deg)));
            this->pose = updated_pose;
            this->speed = floats[3];
            this->accel = floats[4];
            this->ang_speed_deg = floats[5];
            this->ang_accel_deg = floats[6];
        } else {
            printf("OdometrySerial: Invalid COBS encoding\n");
            return {};
        }
    } else if (packet_length == -1) {
        printf("OdometrySerial: Buffer overflow\n");
        return {};
    }
    return pose;
}

void OdometrySerial::set_position(const Pose2d &new_pose) { pose_offset = new_pose; }

Pose2d OdometrySerial::get_position(void) {
    return get_pose2d();
}

Pose2d OdometrySerial::get_pose2d(void) { return pose.relative_to(pose_offset); }

size_t OdometrySerial::cobs_encode(const void *data, size_t length, uint8_t *buffer) {
    assert(data && buffer);

    uint8_t *encode = buffer;
    uint8_t *codep = encode++;
    uint8_t code = 1;

    for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte) {
        if (*byte)
            *encode++ = *byte, ++code;

        if (!*byte || code == 0xff) {
            *codep = code, code = 1, codep = encode;
            if (!*byte || length)
                ++encode;
        }
    }
    *codep = code;

    return (size_t)(encode - buffer);
}

size_t OdometrySerial::cobs_decode(const uint8_t *buffer, size_t length, void *data) {
    assert(buffer && data);

    const uint8_t *byte = buffer;
    uint8_t *decode = (uint8_t *)data;

    for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block) {
        if (block)
            *decode++ = *byte++;
        else {
            block = *byte++;
            if (block && (code != 0xff))
                *decode++ = 0;
            code = block;
            if (!code)
                break;
        }
    }

    return (size_t)(decode - (uint8_t *)data);
}

double OdometrySerial::get_speed() {
    return speed;
}

double OdometrySerial::get_accel() {
    return accel;
}
