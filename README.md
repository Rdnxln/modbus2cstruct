# modbus2cstruct

Convert any number of Modbus HR (Holding Registers) and IR (Input Registers) into static C structures. 

The conversion logic is defined by a flexible set of rules stored in a configuration file. This tool is designed to normalize data from various Modbus sources into your standard application data bus.

## Features
* **Register Support:** Currently supports HR and IR registers.
* **Seamless Integration:** Designed to be directly embedded into your C code.
* **High Performance:** Rules are parsed only once at startup. Evaluation during runtime cycles is extremely fast and involves no re-parsing.

## Integration Steps

You can integrate `modbus2cstruct` into your application in three steps:

1. **Define the Mapping:** Describe your C-struct/typedef (e.g., `AppStruct`) using a `field_map_t field_map[]` array.
2. **Write the Rules:** Create a configuration file with conversion expressions (e.g., `AppStruct.YourField = YourRegister[Num]`).
3. **Initialize and Run:** Add code to parse the rules once at application startup, then evaluate the expressions in your execution loops to populate your C-struct.

## Data Flow Diagram

```text
{ Application processing your AppStruct }
                ^
         [ Data in AppStruct ]
                |
        { modbus2cstruct } <------- [ Set of rules: config-file 1 ]
                ^
    [ HR-, IR-registers data ]
    (Modbus RTU or Modbus TCP)
                |
         { Your Device 1 }

 ------------------------------------------------------------------

{ Application processing your AppStruct }
                ^
         [ Data in AppStruct ]
                |
        { modbus2cstruct } <------- [ Set of rules: config-file N ]
                ^
    [ HR-, IR-registers data ]
    (Modbus RTU or Modbus TCP)
                |
         { Your Device N }
```

## Configuration Rules Example

Examples of conversion rules for registers to populate your `AppStruct`:

```txt
AppStruct.Value = FLOAT((HR[0] << 16) + HR[1])
AppStruct.Code = (IR[16] & 0x0FF0) >> 4
tempVAR = FLOAT((HR[2] << 16) + HR[3])
AppStruct.state.A1 = VAR1 > 434 ? 1 : 0
AppStruct.state.A2 = IR[0] & 1
AppStruct.state.A3 = VAR1 > 434 ? 1 : 0
VAR2 = cos(tempVAR)
AppStruct.SourceTime = IR[301] | (IR[300] << 16)
```

## Code Example

### 1. Your Target C Structure
```c
#include <stdint.h>

typedef struct {
    float Value;         /* Measurement data */
    uint16_t Code;       /* Unit code of value */
    struct {
        uint8_t A1: 1;   /* Bit 1: Quality code 1 */
        uint8_t A2: 1;   /* Bit 2: Quality code 2 */
        uint8_t A3: 1;   /* Bit 3: Quality code 3 */
        uint8_t A4: 4;   /* Bits 4-7: Quality code 4 */
        uint8_t   : 1;   /* Reserved */
        uint8_t B1: 2;   /* Bits 9-10: Quality code N-1 */
        uint8_t B2: 6;   /* Bits 11-16: Quality code N */
    } state;
    uint64_t SourceTime;
} AppStruct;
```

### 2. Structure Mapping Array
```c
#include <stddef.h>

static const field_map_t field_map[] = {
    /* Aliases for 'Value' field */
    {"Value",           offsetof(AppStruct, Value),      FTYPE_FLOAT,    0,  0},
    {"AppStruct.Value", offsetof(AppStruct, Value),      FTYPE_FLOAT,    0,  0},
    {"V",               offsetof(AppStruct, Value),      FTYPE_FLOAT,    0,  0},
    
    /* Standard fields */
    {"Code",            offsetof(AppStruct, Code),       FTYPE_UINT16,   0,  0},
    {"SourceTime",      offsetof(AppStruct, SourceTime), FTYPE_UINT64,   0,  0},
    
    /* Bitfields (Name, Offset, Type, Bit Offset, Bit Width) */
    {"state.A1",        offsetof(AppStruct, state),      FTYPE_BITFIELD, 0,  1},
    {"state.A2",        offsetof(AppStruct, state),      FTYPE_BITFIELD, 1,  1},
    {"state.A3",        offsetof(AppStruct, state),      FTYPE_BITFIELD, 2,  1},
    {"state.A4",        offsetof(AppStruct, state),      FTYPE_BITFIELD, 3,  4},
    {"state.B1",        offsetof(AppStruct, state),      FTYPE_BITFIELD, 8,  2},
    {"state.B2",        offsetof(AppStruct, state),      FTYPE_BITFIELD, 10, 6},
};

#define FIELD_MAP_SIZE (sizeof(field_map) / sizeof(field_map[0]))
```

To Be Continued
