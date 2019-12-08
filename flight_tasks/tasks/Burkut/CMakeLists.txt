px4_add_library(FlightTaskBurkut
	FlightTaskBurkut.cpp
)
target_link_libraries(FlightTaskBurkut PUBLIC FlightTask)
target_include_directories(FlightTaskBurkut PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
