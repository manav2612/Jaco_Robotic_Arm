Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0
Start-Service sshd
Set-Service -Name sshd -StartupType 'Automatic'
New-NetFirewallRule -Name sshd -DisplayName 'OpenSSH Server (sshd)' -Enabled True -Direction Inbound -Protocol TCP -Action Allow -LocalPort 22

New-Item -ItemType Directory -Force -Path C:\ProgramData\ssh | Out-Null
Set-Content -Path C:\ProgramData\ssh\administrators_authorized_keys -Value "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAILcC5QWeY7nwuqHPLApFHirqXWCM9UrFrbejbep4Tnuz jaco-ec2-remote" -Encoding ASCII

icacls.exe C:\ProgramData\ssh\administrators_authorized_keys /inheritance:r
icacls.exe C:\ProgramData\ssh\administrators_authorized_keys /grant "SYSTEM:(F)"
icacls.exe C:\ProgramData\ssh\administrators_authorized_keys /grant "BUILTIN\Administrators:(F)"

Restart-Service sshd
Write-Host "SSH setup complete"
