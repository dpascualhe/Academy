
if( ICE_PY_MODULES AND NUMPY_MODULES AND OpenCV_PY_MODULES AND QT4_PY_MODULES AND QWT5_PY_MODULES)
	#install
	#INSTALL (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION share/jderobot/python/introrob_py
	#	COMPONENT core
	#	FILES_MATCHING
	#		PATTERN "*.py"
	#)

	#INSTALL (PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/introrob_py DESTINATION bin OPTIONAL COMPONENT core)

endif(ICE_PY_MODULES AND NUMPY_MODULES AND OpenCV_PY_MODULES AND QT4_PY_MODULES AND QWT5_PY_MODULES)


