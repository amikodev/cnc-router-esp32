
set(app_sources
	"main.cpp"
    "Axe.cpp"
    "Geometry.cpp"
    "ActionMove.cpp"
	"StepDriver.cpp"
	"CncRouter.cpp"
    "gcode/GCode.cpp"
    "gcode/GCodeCR.cpp"
    "Plasma.cpp"
    "CoordSystem.cpp"

)

set(includes
	"."
)

set(reqs
    spiffs
)

set(files
    ../server/index.html
    ../server/style.css
    ../server/main.js
    ../server/error.html
)


idf_component_register(SRCS ${app_sources} INCLUDE_DIRS ${includes} REQUIRES ${reqs} EMBED_FILES ${files})
