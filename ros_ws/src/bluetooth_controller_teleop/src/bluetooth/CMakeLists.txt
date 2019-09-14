## Bluetooth module

set(BLUETOOTH_SRC
        ${CMAKE_CURRENT_SOURCE_DIR}/bluetooth_test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/BluetoothManager.cpp
        PARENT_SCOPE
)
set(HEADERS
        ${HEADERS}
        include/BluetoothManager.hpp
        PARENT_SCOPE
)