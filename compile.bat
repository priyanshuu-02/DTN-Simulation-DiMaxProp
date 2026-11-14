set targetdir=target

IF NOT EXIST "%targetdir%" mkdir %targetdir%

javac -encoding UTF-8 -sourcepath src -d %targetdir% -cp lib/ECLA.jar;lib/DTNConsoleConnection.jar src/core/*.java src/movement/*.java src/movement/map/*.java src/report/*.java src/routing/*.java src/routing/maxprop/*.java src/gui/*.java src/gui/playfield/*.java src/gui/nodefilter/*.java src/input/*.java src/applications/*.java src/interfaces/*.java

IF NOT EXIST "%targetdir%\gui\buttonGraphics" (
	mkdir %targetdir%\gui\buttonGraphics
	copy src\gui\buttonGraphics\* %targetdir%\gui\buttonGraphics\
)
