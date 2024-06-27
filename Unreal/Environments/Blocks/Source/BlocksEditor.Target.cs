// Copyright 1998-2017 Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;
using System.Collections.Generic;

public class BlocksEditorTarget : TargetRules
{
	public BlocksEditorTarget(TargetInfo Target) : base(Target)
	{
        DefaultBuildSettings = BuildSettingsVersion.V5;
        Type = TargetType.Editor;
		ExtraModuleNames.AddRange(new string[] { "Blocks" });
<<<<<<< HEAD
        DefaultBuildSettings = BuildSettingsVersion.V4;
=======
>>>>>>> f5f6cc1d17237900be5e04cfe99ceb9293f1b14b
        //bUseUnityBuild = false;
        //bUsePCHFiles = false;
    }
}
