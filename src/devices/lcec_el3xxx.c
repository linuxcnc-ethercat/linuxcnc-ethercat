//
//    Copyright (C) 2011 Sascha Ittner <sascha.ittner@modusoft.de>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

/// @file
/// @brief Driver for Beckhoff EL3xxx Analog input modules

#include "../lcec.h"
#include "lcec_class_ain.h"

// TODO(scottlaird): add support for additional EL31xx operating modes, as per
//   https://download.beckhoff.com/download/Document/io/ethercat-terminals/el31xxen.pdf#page=251
//   Specifically, support disabling the filter (0x8000:06) and DC mode.
//
// TODO(scottlaird): using generic support, add EL3255 pot input.
//
// TODO(scottlaird): Figure out what to do with older EL31xx devices
//   without 0x6000:e (the sync error PDO).  It looks like it was
//   added in r18.  Is there a point in keeping the sync error pin at all?

#define LCEC_EL3XXX_MODPARAM_SENSOR     0
#define LCEC_EL3XXX_MODPARAM_RESOLUTION 8
#define LCEC_EL3XXX_MODPARAM_WIRES      16

#define LCEC_EL3XXX_MAXCHANS 8  // for sizing arrays

static int lcec_el3xxx_init(int comp_id, lcec_slave_t *slave);

/// @brief Modparams settings available via XML.
static const lcec_modparam_desc_t modparams_temperature[] = {
    {"ch0Sensor", LCEC_EL3XXX_MODPARAM_SENSOR + 0, MODPARAM_TYPE_STRING},
    {"ch1Sensor", LCEC_EL3XXX_MODPARAM_SENSOR + 1, MODPARAM_TYPE_STRING},
    {"ch2Sensor", LCEC_EL3XXX_MODPARAM_SENSOR + 2, MODPARAM_TYPE_STRING},
    {"ch3Sensor", LCEC_EL3XXX_MODPARAM_SENSOR + 3, MODPARAM_TYPE_STRING},
    {"ch4Sensor", LCEC_EL3XXX_MODPARAM_SENSOR + 4, MODPARAM_TYPE_STRING},
    {"ch5Sensor", LCEC_EL3XXX_MODPARAM_SENSOR + 5, MODPARAM_TYPE_STRING},
    {"ch6Sensor", LCEC_EL3XXX_MODPARAM_SENSOR + 6, MODPARAM_TYPE_STRING},
    {"ch7Sensor", LCEC_EL3XXX_MODPARAM_SENSOR + 7, MODPARAM_TYPE_STRING},
    {"ch0Resolution", LCEC_EL3XXX_MODPARAM_RESOLUTION + 0, MODPARAM_TYPE_STRING},
    {"ch1Resolution", LCEC_EL3XXX_MODPARAM_RESOLUTION + 1, MODPARAM_TYPE_STRING},
    {"ch2Resolution", LCEC_EL3XXX_MODPARAM_RESOLUTION + 2, MODPARAM_TYPE_STRING},
    {"ch3Resolution", LCEC_EL3XXX_MODPARAM_RESOLUTION + 3, MODPARAM_TYPE_STRING},
    {"ch4Resolution", LCEC_EL3XXX_MODPARAM_RESOLUTION + 4, MODPARAM_TYPE_STRING},
    {"ch5Resolution", LCEC_EL3XXX_MODPARAM_RESOLUTION + 5, MODPARAM_TYPE_STRING},
    {"ch6Resolution", LCEC_EL3XXX_MODPARAM_RESOLUTION + 6, MODPARAM_TYPE_STRING},
    {"ch7Resolution", LCEC_EL3XXX_MODPARAM_RESOLUTION + 7, MODPARAM_TYPE_STRING},
    {"ch0Wires", LCEC_EL3XXX_MODPARAM_WIRES + 0, MODPARAM_TYPE_STRING},
    {"ch1Wires", LCEC_EL3XXX_MODPARAM_WIRES + 1, MODPARAM_TYPE_STRING},
    {"ch2Wires", LCEC_EL3XXX_MODPARAM_WIRES + 2, MODPARAM_TYPE_STRING},
    {"ch3Wires", LCEC_EL3XXX_MODPARAM_WIRES + 3, MODPARAM_TYPE_STRING},
    {"ch4Wires", LCEC_EL3XXX_MODPARAM_WIRES + 4, MODPARAM_TYPE_STRING},
    {"ch5Wires", LCEC_EL3XXX_MODPARAM_WIRES + 5, MODPARAM_TYPE_STRING},
    {"ch6Wires", LCEC_EL3XXX_MODPARAM_WIRES + 6, MODPARAM_TYPE_STRING},
    {"ch7Wires", LCEC_EL3XXX_MODPARAM_WIRES + 7, MODPARAM_TYPE_STRING},
    {NULL},
};

/// @brief Lookup table of known temperature sensor types and their codes.
///
/// From https://download.beckhoff.com/download/Document/io/ethercat-terminals/el32xxen.pdf#page=223
static const lcec_lookuptable_int_t temp_sensors_setting[] = {
    {"Pt100", 0},          // Pt100 sensor,
    {"Ni100", 1},          // Ni100 sensor, -60 to 250C
    {"Pt1000", 2},         // Pt1000 sensor, -200 to 850C
    {"Pt500", 3},          // Pt500 sensor, -200 to 850C
    {"Pt200", 4},          // Pt200 sensor, -200 to 850C
    {"Ni1000", 5},         // Ni1000 sensor, -60 to 250C
    {"Ni1000-TK5000", 6},  // Ni1000-TK5000, -30 to 160C
    {"Ni120", 7},          // Ni120 sensor, -60 to 320C
    {"Ohm/16", 8},         // no sensor, report Ohms directly.  0-4095 Ohms
    {"Ohm/64", 9},         // no sensor, report Ohms directly.  0-1023 Ohms
    {NULL},
};

/// @brief Lookup table of known temperature sensor types that return unsigned values.
static const lcec_lookuptable_int_t temp_sensors_unsigned[] = {
    {"Ohm/16", 1},
    {"Ohm/64", 1},
    {NULL},
};

/// @brief Lookup table of known temperature senso types with non-default scales.
static const lcec_lookuptable_double_t temp_sensors_scale[] = {
    {"Ohm/16", 1.0 / 16},
    {"Ohm/64", 1.0 / 64},
    {NULL},
};

/// @brief Lookup table of available values for the `resolutions` modParam setting.
///
/// From https://download.beckhoff.com/download/Document/io/ethercat-terminals/el32xxen.pdf#page=222
static const lcec_lookuptable_int_t temp_resolutions_setting[] = {
    {"Signed", 0},    // 0.1C per bit, default on most devices
    {"Standard", 0},  // Same as "signed", but easier to remember WRT "High".
                      // { "Absolute", 1, 1.0 }, // ones-compliment presentation, why?
    {"High", 2},      // 0.01C per bit, default on "high precision" devices.
    {NULL},
};

/// @brief Lookup table of non-default scale values for resolution settings
static const lcec_lookuptable_double_t temp_resolutions_scale[] = {
    {"High", 0.1},  // 0.01C per bit, default on "high precision" devices.
    {NULL},
};

/// @brief Lookup table of available values for the `wires` modParam setting.
static const lcec_lookuptable_int_t temp_wires[] = {
    {"2", 0},
    {"3", 1},
    {"4", 2},
    {NULL},
};

/// Flags for describing devices
#define F_CHANNELS(x) (x)      ///< Number of input channels
#define F_SYNC        1 << 14  ///< Device has `sync-error` PDO
#define F_TEMPERATURE 1 << 15  ///< Device is a temperature sensor
#define F_PRESSURE    1 << 16  ///< Device is a pressure sensor

#define INPORTS(flag) ((flag)&0xf)  // Number of input channels
#define PDOS(flag)    (((flag)&F_SYNC) ? (5 * INPORTS(flag)) : (4 * INPORTS(flag)))

/// Macro to avoid repeating all of the unchanging fields in
/// `lcec_typelist_t`.
#define BECKHOFF_AIN_DEVICE(name, pid, flags) \
  { name, LCEC_BECKHOFF_VID, pid, 0, NULL, lcec_el3xxx_init, NULL, flags }

/// Macro for defining devices that take `<modParam>`s in the XML config.
#define BECKHOFF_AIN_DEVICE_PARAMS(name, pid, flags, modparams) \
  { name, LCEC_BECKHOFF_VID, pid, 0, NULL, lcec_el3xxx_init, modparams, flags }

/// @brief Devices supported by this driver.
static lcec_typelist_t types[] = {
    // 12-bit devices
    BECKHOFF_AIN_DEVICE("EL3001", 0x0bb93052, F_CHANNELS(1)),
    BECKHOFF_AIN_DEVICE("EL3002", 0x0bba3052, F_CHANNELS(2)),
    BECKHOFF_AIN_DEVICE("EL3004", 0x0bbc3052, F_CHANNELS(4)),
    BECKHOFF_AIN_DEVICE("EL3008", 0x0bc03052, F_CHANNELS(8)),
    BECKHOFF_AIN_DEVICE("EL3011", 0x0bc33052, F_CHANNELS(1)),
    BECKHOFF_AIN_DEVICE("EL3012", 0x0bc43052, F_CHANNELS(2)),
    BECKHOFF_AIN_DEVICE("EL3014", 0x0bc63052, F_CHANNELS(3)),
    BECKHOFF_AIN_DEVICE("EL3021", 0x0bcd3052, F_CHANNELS(1)),
    BECKHOFF_AIN_DEVICE("EL3022", 0x0bce3052, F_CHANNELS(2)),
    BECKHOFF_AIN_DEVICE("EL3024", 0x0bd03052, F_CHANNELS(4)),
    BECKHOFF_AIN_DEVICE("EL3041", 0x0be13052, F_CHANNELS(1)),
    BECKHOFF_AIN_DEVICE("EL3042", 0x0be23052, F_CHANNELS(2)),
    BECKHOFF_AIN_DEVICE("EL3044", 0x0be43052, F_CHANNELS(4)),
    BECKHOFF_AIN_DEVICE("EL3048", 0x0be83052, F_CHANNELS(8)),
    BECKHOFF_AIN_DEVICE("EL3051", 0x0beb3052, F_CHANNELS(1)),
    BECKHOFF_AIN_DEVICE("EL3052", 0x0bec3052, F_CHANNELS(2)),
    BECKHOFF_AIN_DEVICE("EL3054", 0x0bee3052, F_CHANNELS(4)),
    BECKHOFF_AIN_DEVICE("EL3058", 0x0bf23052, F_CHANNELS(8)),
    BECKHOFF_AIN_DEVICE("EL3061", 0x0bf53052, F_CHANNELS(1)),
    BECKHOFF_AIN_DEVICE("EL3062", 0x0bf63052, F_CHANNELS(2)),
    BECKHOFF_AIN_DEVICE("EL3064", 0x0bf83052, F_CHANNELS(4)),
    BECKHOFF_AIN_DEVICE("EL3068", 0x0bfc3052, F_CHANNELS(8)),
    BECKHOFF_AIN_DEVICE("EJ3004", 0x0bbc2852, F_CHANNELS(4)),

    // 16-bit devices.  These include a `sync-err` PDO that 12-bit devices lack.
    BECKHOFF_AIN_DEVICE("EL3101", 0x0c1d3052, F_CHANNELS(1) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3102", 0x0c1e3052, F_CHANNELS(2) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3104", 0x0c203052, F_CHANNELS(4) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3111", 0x0c273052, F_CHANNELS(1) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3112", 0x0c283052, F_CHANNELS(2) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3114", 0x0c2a3052, F_CHANNELS(4) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3121", 0x0c313052, F_CHANNELS(1) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3122", 0x0c323052, F_CHANNELS(2) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3124", 0x0c343052, F_CHANNELS(4) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3141", 0x0c453052, F_CHANNELS(1) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3142", 0x0c463052, F_CHANNELS(2) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3144", 0x0c483052, F_CHANNELS(4) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3151", 0x0c4f3052, F_CHANNELS(1) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3152", 0x0c503052, F_CHANNELS(2) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3154", 0x0c523052, F_CHANNELS(4) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3161", 0x0c593052, F_CHANNELS(1) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3162", 0x0c5a3052, F_CHANNELS(2) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3164", 0x0c5c3052, F_CHANNELS(4) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EL3182", 0x0c6e3052, F_CHANNELS(2) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EP3174", 0x0c664052, F_CHANNELS(4) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EP3184", 0x0c704052, F_CHANNELS(4) | F_SYNC),
    BECKHOFF_AIN_DEVICE("EPX3158", 0x9809ab69, F_CHANNELS(8) | F_SYNC),

    // Temperature devices.  They need to include `modparams_temperature` to allow for sensor-type settings.
    BECKHOFF_AIN_DEVICE_PARAMS("EJ3202", 0x0c822852, F_CHANNELS(2) | F_TEMPERATURE, modparams_temperature),
    BECKHOFF_AIN_DEVICE_PARAMS("EJ3214", 0x0c8e2852, F_CHANNELS(4) | F_TEMPERATURE, modparams_temperature),
    BECKHOFF_AIN_DEVICE_PARAMS("EL3201", 0x0c813052, F_CHANNELS(1) | F_TEMPERATURE, modparams_temperature),
    BECKHOFF_AIN_DEVICE_PARAMS("EL3202", 0x0c823052, F_CHANNELS(2) | F_TEMPERATURE, modparams_temperature),
    BECKHOFF_AIN_DEVICE_PARAMS("EL3204", 0x0c843052, F_CHANNELS(4) | F_TEMPERATURE, modparams_temperature),
    BECKHOFF_AIN_DEVICE_PARAMS("EL3208", 0x0c883052, F_CHANNELS(8) | F_TEMPERATURE, modparams_temperature),
    BECKHOFF_AIN_DEVICE_PARAMS("EL3214", 0x0c8e3052, F_CHANNELS(4) | F_TEMPERATURE, modparams_temperature),
    BECKHOFF_AIN_DEVICE_PARAMS("EL3218", 0x0c923052, F_CHANNELS(8) | F_TEMPERATURE, modparams_temperature),
    BECKHOFF_AIN_DEVICE_PARAMS("EP3204", 0x0c844052, F_CHANNELS(4) | F_TEMPERATURE, modparams_temperature),

    // Pressure sensors.
    BECKHOFF_AIN_DEVICE("EM3701", 0x0e753452, F_CHANNELS(1) | F_PRESSURE),
    BECKHOFF_AIN_DEVICE("EM3702", 0x0e763452, F_CHANNELS(2) | F_PRESSURE),
    BECKHOFF_AIN_DEVICE("EM3712", 0x0e803452, F_CHANNELS(2) | F_PRESSURE),
    {NULL},
};
ADD_TYPES(types)

static void lcec_el3xxx_read(lcec_slave_t *slave, long period);
static int set_sensor_type(lcec_slave_t *slave, char *sensortype, lcec_class_ain_channel_t *chan, int idx, int sidx);
static int set_resolution(lcec_slave_t *slave, char *resolution_name, lcec_class_ain_channel_t *chan, int idx, int sidx);
static int set_wires(lcec_slave_t *slave, char *wires_name, lcec_class_ain_channel_t *chan, int idx, int sidx);

/// @brief Initialize an EL3xxx device.
static int lcec_el3xxx_init(int comp_id, lcec_slave_t *slave) {
  lcec_class_ain_channels_t *hal_data;
  uint64_t flags;

  flags = slave->flags;

  rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "initing device as %s, flags %lx\n", slave->name, flags);
  rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "- slave is %p\n", slave);

  hal_data = lcec_ain_allocate_channels(INPORTS(slave->flags));
  slave->hal_data = hal_data;

  for (int i = 0; i < hal_data->count; i++) {
    lcec_class_ain_options_t *options = lcec_ain_options();
    options->has_sync = flags & F_SYNC;
    options->is_temperature = flags & F_TEMPERATURE;
    options->is_pressure = flags & F_PRESSURE;

    hal_data->channels[i] = lcec_ain_register_channel(slave, i, 0x6000 + (i << 4), options);
  }

  slave->proc_read = lcec_el3xxx_read;

  // handle modParams
  for (int i = 0; i < hal_data->count; i++) {
    lcec_class_ain_channel_t *chan = hal_data->channels[i];

    // Handle modparams
    if (flags & F_TEMPERATURE) {
      LCEC_CONF_MODPARAM_VAL_T *pval;

      // <modParam name="chXSensor" value="???"/>
      pval = lcec_modparam_get(slave, LCEC_EL3XXX_MODPARAM_SENSOR + i);
      if (pval != NULL) {
        if (set_sensor_type(slave, pval->str, chan, 0x8000 + (i << 4), 0x19) != 0)
          return -1;  // set_sensor_type logs an error message so we don't have to.
      }

      // <modParam name="chXResolution" value="???"/>
      pval = lcec_modparam_get(slave, LCEC_EL3XXX_MODPARAM_RESOLUTION + i);
      if (pval != NULL) {
        if (set_resolution(slave, pval->str, chan, 0x8000 + (i << 4), 0x2) != 0)
          return -1;  // set_resolution logs an error message so we don't have to.
      }

      // <modParam name="chXWires", value="???"/>
      pval = lcec_modparam_get(slave, LCEC_EL3XXX_MODPARAM_WIRES + i);
      if (pval != NULL) {
        if (set_wires(slave, pval->str, chan, 0x8000 + (i << 4), 0x1a) != 0)
          return -1;  // set_resolution logs an error message so we don't have to.
      }
    }
  }
  rtapi_print_msg(RTAPI_MSG_DBG, LCEC_MSG_PFX "done\n");

  return 0;
}

/// @brief Read values from the device.
static void lcec_el3xxx_read(lcec_slave_t *slave, long period) {
  lcec_class_ain_channels_t *hal_data = (lcec_class_ain_channels_t *)slave->hal_data;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  lcec_ain_read_all(slave, hal_data);
}

/// @brief Set the sensor type for a channel.
static int set_sensor_type(lcec_slave_t *slave, char *sensortype, lcec_class_ain_channel_t *chan, int idx, int sidx) {
  int setting = lcec_lookupint_i(temp_sensors_setting, sensortype, -1);

  if (setting != -1) {
    // See if this sensor type needs a non-default scale, otherwise default to 0.1.
    *(chan->scale) = lcec_lookupdouble_i(temp_sensors_scale, sensortype, 0.1);

    // See if this sensor type is unsigned.  Otherwise signed.
    chan->is_unsigned = lcec_lookupint_i(temp_sensors_unsigned, sensortype, 0);
    if (lcec_write_sdo16(slave, idx, sidx, setting) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to configure sensor for slave %s.%s\n", slave->master->name, slave->name);
      return -1;
    }
  } else {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "unknown sensor type \"%s\"\n", sensortype);
    return -1;
  }

  return 0;
}

/// @brief Set the resolution for a channel.
static int set_resolution(lcec_slave_t *slave, char *resolution_name, lcec_class_ain_channel_t *chan, int idx, int sidx) {
  int setting = lcec_lookupint_i(temp_resolutions_setting, resolution_name, -1);

  if (setting != -1) {
    // See if this resolution type needs a non-default scale, otherwise leave it at 1.0.
    *(chan->scale) = *(chan->scale) * lcec_lookupdouble_i(temp_resolutions_scale, resolution_name, 1.0);
    if (lcec_write_sdo8(slave, idx, sidx, setting) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to configure slave %s.%s sdo resolution!\n", slave->master->name, slave->name);
      return -1;
    }
  } else {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "unknown resolution \"%s\"\n", resolution_name);
    return -1;
  }
  return 0;
}

/// @brief Set the wire count for a channel.
static int set_wires(lcec_slave_t *slave, char *wires_name, lcec_class_ain_channel_t *chan, int idx, int sidx) {
  int wirevalue;

  wirevalue = lcec_lookupint_i(temp_wires, wires_name, -1);
  if (wirevalue != -1) {
    if (lcec_write_sdo16(slave, idx, sidx, wirevalue) != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "failed to configure slave %s.%s sdo wires!\n", slave->master->name, slave->name);
      return -1;
    }
  } else {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "unknown wire setting \"%s\"\n", wires_name);
    return -1;
  }
  return 0;
}
