{
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  
  };

  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in {
        devShells.default = pkgs.mkShell {
          name = "Example project";
          packages = [
            pkgs.colcon
            pkgs.python313
            pkgs.python313Packages.scipy
            pkgs.python313Packages.numpy
            pkgs.python313Packages.matplotlib
            (with pkgs.rosPackages.jazzy; buildEnv {
              underlay = true;
              paths = [
                ros-core
                rviz2
                ros-gz
                ros-gz-sim
                ros-gz-bridge
                ros-gz-image
              ];
            })
          ];
          shellHook = '' source ${pkgs.rosPackages.jazzy.ros-core}/setup.bash'';
        };
      });
  
}