workspace "FFF-DT"
	architecture "x64"
	startproject "app"

	configurations{
		"Release"
	}
	
	flags
	{
		"MultiProcessorCompile"
	}

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"
solutiondir = path.getabsolute("./")

IncludeDir = {}
IncludeDir["merlin"] = "merlin-lib/include"
IncludeDir["glfw"] = "merlin-lib/vendor/glfw/include"
IncludeDir["glad"] = "merlin-lib/vendor/glad/include"
IncludeDir["imgui"] = "merlin-lib/vendor/imgui"
IncludeDir["glm"] = "merlin-lib/vendor/glm"
IncludeDir["stb_image"] = "merlin-lib/vendor/stb_image"
IncludeDir["tinyfiledialogs"] = "merlin-lib/vendor/tinyfiledialogs"
IncludeDir["tinyxml2"] = "merlin-lib/vendor/tinyxml2"
IncludeDir["ImGuiTextEditor"] = "merlin-lib/vendor/ImGuiTextEditor"
IncludeDir["assimp"] = "merlin-lib/vendor/assimp/include"

project "app"
	kind "ConsoleApp"
	language "C++"
	cppdialect "C++20"
	staticruntime "on"
	debugdir ("./")
	targetdir (solutiondir .."/bin/" .. outputdir .. "/%{prj.name}")
	objdir (solutiondir .."/bin-int/" .. outputdir .. "/%{prj.name}")

	files
	{
		"src/**.h",
		"src/**.cpp",
		"assets/shaders/**.*"
	}

	defines
	{
		"_CRT_SECURE_NO_WARNINGS",
		"GLM_ENABLE_EXPERIMENTAL"
	}
	
	removefiles { "assets/common/**.*" }

	vpaths {
	   ["Headers/*"] = "**.h",
	   ["Sources/*"] = {"**.c", "**.cpp"},
	   ["Docs"] = "**.md",
	   ["Assets/*"] = "assets/**.*"
	}

	includedirs {  }

	filter { "system:windows" }
		ignoredefaultlibraries { "msvcrt" }

	includedirs
	{
		solutiondir .. "merlin-lib/vendor",
		solutiondir .. "/%{IncludeDir.merlin}",
		solutiondir .. "/%{IncludeDir.glm}",
		solutiondir .. "/%{IncludeDir.glad}",
		solutiondir .. "/%{IncludeDir.tinyfiledialogs}",
		solutiondir .. "/%{IncludeDir.imgui}",
		solutiondir .. "/%{IncludeDir.glfw}"
	}
	
	links
	{
		solutiondir .."/merlin-lib/lib/merlin",
		solutiondir .."/merlin-lib/lib/glad",
		solutiondir .."/merlin-lib/lib/glfw",
		solutiondir .."/merlin-lib/lib/imgui",
		solutiondir .."/merlin-lib/lib/assimp",
		solutiondir .."/merlin-lib/lib/stb_image",
		solutiondir .."/merlin-lib/lib/tinyfiledialogs",
		"opengl32"
	}

	filter "system:windows"
		systemversion "latest"

		defines
		{
			"GLCORE_PLATFORM_WINDOWS",
			"NOMINMAX"
		}

	filter "configurations:Debug"
		defines "GLCORE_DEBUG"
		runtime "Debug"
		symbols "on"
		postbuildcommands {
		  "{COPYDIR} %[assets] %[%{!cfg.targetdir}/assets]",
		  "{COPYDIR} %[" .. solutiondir .. "/merlin-lib/assets] %[%{!cfg.debugdir}/assets/common]"
		}

	filter "configurations:Release"
		defines "GLCORE_RELEASE"
		runtime "Release"
		optimize "on"
		postbuildcommands {
		  "{COPYDIR} %[assets] %[%{!cfg.targetdir}/assets]",
		  "{COPYDIR} %[" .. solutiondir .. "/merlin-lib/assets] %[%{!cfg.debugdir}/assets/common]"
		}