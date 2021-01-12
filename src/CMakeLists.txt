
set(app_sources
	"main.cpp"
	"StepDriver.cpp"
	"CncRouter.cpp"
    "CncRouter_gcode.cpp"
    "Plasma.cpp"
)

set(includes
	"."
)

set(reqs
)

set(files
)


idf_component_register(SRCS ${app_sources} INCLUDE_DIRS ${includes} REQUIRES ${reqs} EMBED_FILES ${files})
