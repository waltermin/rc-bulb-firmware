{
  description = "BKJIG2: ESP32-S3 flashing harness for the T1-2S-NL (BK7238)";

  # Same pins as rclamp's rgbcw-basestation (flake.lock copied from there), so
  # the ESP-IDF toolchain is shared from the nix store.
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev";
  };

  outputs = { self, nixpkgs, esp-dev }:
    let
      system = "aarch64-darwin";
      pkgs = import nixpkgs { inherit system; };
    in {
      devShells.${system}.default = pkgs.mkShell {
        name = "bkjig2";
        buildInputs = [
          esp-dev.packages.${system}.esp-idf-esp32s3
          pkgs.python3
          pkgs.python3Packages.pyserial
        ];
      };
    };
}
