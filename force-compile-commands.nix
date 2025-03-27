# GOAL
# what I want to do: given a derivation, i want to go through all of its dependencies 
# and add a cmake flag to them within an overlay. I will append these overlays to a list
# and then combine them all into one overlay to apply.

# REALITY
# right now what this can do is given a list of deps within a derivation you can make them have the compile command

{ lib } : 
let
  forceCompileCommands = drv: drv.overrideAttrs (finalAttrs: previousAttrs: {
    cmakeFlags = (previousAttrs.cmakeFlags or []) ++ [ "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" ];
  });
  applyForceCC = deps : map forceCompileCommands deps;
in
{
  inherit forceCompileCommands applyForceCC;
}