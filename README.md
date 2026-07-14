# modbus2cstruct
Convert any Modbus registers into static C structures.
The conversion logic is defined by a flexible set of rules stored in a configuration file.
Designed to normalize data from various Modbus-sources into your standard application data bus.

The only HR-, IR- registers are supported now.


``` TXT
{ Application }
 ^- Data in AppStruct -< { modbus2cstruct }             <- Set of rules (config-file 1)
                          ^- Data in HR-, IR-registers
                             (modbus-RTU or Modbus-TCP) <- Your Device 1,
                                                           supported modbus-protocol
                                                           connection

 ^- Data in AppStruct -< { modbus2cstruct }             <- Set of rules (config-file 2) fileN
                          ^- Data in HR-, IR-registers
                             (modbus-RTU or Modbus-TCP) <- Your Device N,
                                                           supported modbus-protocol
                                                           connection
```


Rules for registers convertation to your AppStruct:
``` txt
AppStruct.Value  =  FLOAT(HR[0] << 16 + HR[1])
AppStruct.Code   = ( IR[16] & 0x0FF0 ) >> 4
tempVAR          = FLOAT(HR[2] <<16 + HR[3])
AppStruct.state.A1 = VAR1 > 434 ? 1 : 0
AppStruct.state.A2 = IR[0] & 1
AppStruct.state.A3 = VAR1 > 434 ? 1 : 0
VAR2 = cos(tempVAR)
AppStruct.SourceTime = IR[301] | ( IR[300]<<16 )
```


For example, your application accept data with using specified data structure like this:
``` C
typedef struct {
  float      Value; /* float data,         Data of measurement */
  uint16_t   Code;  /* short integer data, Unit's code of value */
  struct {
    uint8_t  A1: 1; /* bit1 of data,       Quality code 1 */
    uint8_t  A2: 1; /* bit2 of data,       Quality code 2 */
    uint8_t  A3: 1; /* ...                 Quality code 3 */
    uint8_t  A4: 4; /* bit-field of data,  Quality code 4 */
    uint8_t    : 1; /* reserved, not used yet      */
    uint8_t  B1: 2; /* bit-field of data,  Quality code  N-1 */
    uint8_t  B2: 6; /* bit-field of data,  Quality code  N   */
  } state ;
  uint64_t   SourceTime;
} AppStruct ;
```

You can define mapping array for your structure AppStruct:
``` C
static const field_map_t field_map[] = {
    /* Field name */
    {"Value",
    /* Byte offset for field in application struct */
                        offsetof(AppStruct, Value),
    /* Type of field */
                                                    FTYPE_FLOAT,
    /* Number of bit and Bit width (used only for bit or bit-field) */
                                                                 0, 0},
    {"AppStruct.Value", offsetof(AppStruct, Value), FTYPE_FLOAT, 0, 0},
    {"V",               offsetof(AppStruct, Value), FTYPE_FLOAT, 0, 0},
    /* NOTE: 'Value',
             'AppStruct.Value'
             'V' are the names (aliases) for field Value in AppStruct */

    {"Code", offsetof(AppStruct, Code), FTYPE_UINT16, 0, 0},
    {"SourceTime", offsetof(AppStruct, SourceTime), FTYPE_UINT64, 0, 0},

    /* Bit fields */
    {"state.A1", offsetof(AppStruct, state), FTYPE_BITFIELD, 0,  1},
    {"state.A2", offsetof(AppStruct, state), FTYPE_BITFIELD, 1,  1},
    {"state.A3", offsetof(AppStruct, state), FTYPE_BITFIELD, 2,  1},
    {"state.A4", offsetof(AppStruct, state), FTYPE_BITFIELD, 3,  4},
    {"state.B1", offsetof(AppStruct, state), FTYPE_BITFIELD, 8,  2},
    {"state.B2", offsetof(AppStruct, state), FTYPE_BITFIELD, 10, 6},
};
#define FIELD_MAP_SIZE (sizeof(field_map) / sizeof(field_map[0]))
```

TODO: finish this description
