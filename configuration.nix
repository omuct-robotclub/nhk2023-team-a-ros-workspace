# Edit this configuration file to define what should be installed on
# your system.  Help is available in the configuration.nix(5) man page
# and in the NixOS manual (accessible by running ‘nixos-help’).

{ config, pkgs, lib, ... }:

{
  imports =
    [ # Include the results of the hardware scan.
      ./hardware-configuration.nix
    ];

  boot.kernelPackages = pkgs.linuxPackages_latest;

  nix.settings.experimental-features = [ "nix-command" "flakes" ];
  nix.settings.trusted-users = [ "root" "robotclub" ];
  nix.extraOptions = ''
    keep-outputs = true
    keep-derivations = true
  '';


  # Bootloader.
  boot.loader.systemd-boot.enable = true;
  boot.loader.efi.canTouchEfiVariables = true;

  networking.hostName = "robotclub-latitude"; # Define your hostname.
  # networking.wireless.enable = true;  # Enables wireless support via wpa_supplicant.

  # Configure network proxy if necessary
  # networking.proxy.default = "http://user:password@proxy:port/";
  # networking.proxy.noProxy = "127.0.0.1,localhost,internal.domain";

  # Enable networking
  networking.networkmanager.enable = true;

  # Set your time zone.
  time.timeZone = "Asia/Tokyo";

  # Select internationalisation properties.
  i18n.defaultLocale = "ja_JP.UTF-8";

  i18n.extraLocaleSettings = {
    LC_ADDRESS = "ja_JP.UTF-8";
    LC_IDENTIFICATION = "ja_JP.UTF-8";
    LC_MEASUREMENT = "ja_JP.UTF-8";
    LC_MONETARY = "ja_JP.UTF-8";
    LC_NAME = "ja_JP.UTF-8";
    LC_NUMERIC = "ja_JP.UTF-8";
    LC_PAPER = "ja_JP.UTF-8";
    LC_TELEPHONE = "ja_JP.UTF-8";
    LC_TIME = "ja_JP.UTF-8";
  };

  fonts.fonts = with pkgs; [
    noto-fonts
    noto-fonts-cjk
    noto-fonts-emoji
  ];

  # Enable the X11 windowing system.
  #services.xserver.enable = true;

  # Enable the GNOME Desktop Environment.
  #services.xserver.displayManager.gdm.enable = true;
  #services.xserver.desktopManager.gnome.enable = true;

  services.xserver = {
    enable = true;
    desktopManager = {
      xterm.enable = false;
      plasma5.enable = true;
    };
    displayManager.sddm.enable = true;
    #displayManager.defaultSession = "xfce";
  };

  # Configure keymap in X11
  services.xserver = {
    layout = "jp";
    xkbVariant = "";
  };

  # Configure console keymap
  console.keyMap = "jp106";

  # Enable CUPS to print documents.
  services.printing.enable = true;

  # Enable sound with pipewire.
  sound.enable = true;
  hardware.pulseaudio.enable = false;
  security.rtkit.enable = true;
  services.pipewire = {
    enable = true;
    alsa.enable = true;
    alsa.support32Bit = true;
    pulse.enable = true;
    # If you want to use JACK applications, uncomment this
    #jack.enable = true;

    # use the example session manager (no others are packaged yet so this is enabled by default,
    # no need to redefine it in your config for now)
    #media-session.enable = true;
  };

  # Enable touchpad support (enabled default in most desktopManager).
  # services.xserver.libinput.enable = true;

  # Define a user account. Don't forget to set a password with ‘passwd’.
  users.users.robotclub = {
    isNormalUser = true;
    description = "robotclub";
    extraGroups = [ "networkmanager" "wheel" "dialout" ];
    openssh.authorizedKeys.keys = [
      "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABgQD2paFy/h8Z4geIE1PueOW5MZNlYCm8bCSQybBMIA/2i734BnqE0uG4KkpuZ98PEIplHHBeUtkrD4WKKx7pqqpO+iMNR/q5ccMaom47kifMqSLylNNxxX4GTgKg6vMPY2CPLULzoWpxOd+zWWjN7ZHaAQ6iUa/c4EMni5SAVp7fowLMjvuglmdTjJfe8DSodseUzQ+lpDWpCfwgQg/euNMM1AiSoD65+Cu1+mX9Lx5tzKTC8SCR8m04BVxf0xlsVoYzYRg8hpoorBtrI9lkR9okgVH7kDGU3O8M9e5v+jIYHMxrmsCLr4xbPN7VoGo4e2YaPV2rinRyNa3JR8DHbKuBZjwLAnGftFeI7GhxJnM7s0Cst2mBOM3fl9qptNjFIKX0b5Da5wNCJm7gb29ov+CX3oibGWf+T4R4nHSfqaMG7zaIQYuvPV2gz/WhBfKW3shUhM7QzZOTdOfdtjyW5GMSJIX4nujPxyxElgULi5bDMJQZ9qdfaMCt1tS5YEBQdF0= aoi@gpd-win-max"
      "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIKi2yXcURGqBRlImY0p2yKFa1ME2WlILyVhv2ZVHDS7/ teru@11"
    ];
    packages = with pkgs; [
      firefox
    #  thunderbird
    ];
  };

  # Enable automatic login for the user.
  services.xserver.displayManager.autoLogin.enable = true;
  services.xserver.displayManager.autoLogin.user = "robotclub";

  # Workaround for GNOME autologin: https://github.com/NixOS/nixpkgs/issues/103746#issuecomment-945091229
  systemd.services."getty@tty1".enable = false;
  systemd.services."autovt@tty1".enable = false;

  # Allow unfree packages
  nixpkgs.config.allowUnfree = true;

  # List packages installed in system profile. To search, run:
  # $ nix search wget
  environment.systemPackages = with pkgs; [
    git
    cachix
    bottom
    neofetch
    can-utils
    usbutils
    acpi
    bottom
    picocom
  #  vim # Do not forget to add an editor to edit configuration.nix! The Nano editor is also installed by default.
  #  wget
  ];

  programs.nix-ld.enable = true;
  
  systemd.extraConfig = ''
    DefaultTimeoutStopSec=10s
  '';  

  # Some programs need SUID wrappers, can be configured further or are
  # started in user sessions.
  # programs.mtr.enable = true;
  # programs.gnupg.agent = {
  #   enable = true;
  #   enableSSHSupport = true;
  # };

  # List services that you want to enable:

  # Enable the OpenSSH daemon.
  services.openssh.enable = true;

  services.avahi = {
    enable = true;
    publish = {
      enable = true;
      addresses = true;
      workstation = true;
    };
  };

  security.sudo.extraRules = [
    {
      users = [ "robotclub" ];
      commands = [
        {
          command = "ALL" ;
          options= [ "NOPASSWD" ]; # "SETENV" # Adding the following could be a good idea
        }
      ];
    }
  ];

  systemd.network = {
    enable = true;
    wait-online.enable = false;
    networks."80-can" = {
      matchConfig.Name = "can0";
      linkConfig.RequiredForOnline = "no";
      extraConfig = ''
        [CAN]
        BitRate=1M
        RestartSec=100ms
      '';
    };
  };

  services.devmon.enable = true;

  services.logind = {
    lidSwitch = "ignore";
    lidSwitchExternalPower = "ignore";
    lidSwitchDocked = "ignore";
  };

  environment.sessionVariables = {
#    QT_IM_MODULE = "qtvirtualkeyboard";
  };


  # Open ports in the firewall.
  # networking.firewall.allowedTCPPorts = [ ... ];
  # networking.firewall.allowedUDPPorts = [ ... ];
  # Or disable the firewall altogether.
  networking.firewall.enable = false;

  # This value determines the NixOS release from which the default
  # settings for stateful data, like file locations and database versions
  # on your system were taken. It‘s perfectly fine and recommended to leave
  # this value at the release version of the first install of this system.
  # Before changing this value read the documentation for this option
  # (e.g. man configuration.nix or on https://nixos.org/nixos/options.html).
  system.stateVersion = "23.05"; # Did you read the comment?

}
