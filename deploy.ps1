# Deploy reip_node.py to all robots
# Usage: .\deploy.ps1

$robots = @("clanker1", "clanker2", "clanker3", "clanker4", "clanker5")

foreach ($r in $robots) {
    Write-Host "Deploying to $r..." -ForegroundColor Cyan
    scp robot/reip_node.py "pi@${r}.local:~/" 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  OK" -ForegroundColor Green
    } else {
        Write-Host "  Failed (robot offline?)" -ForegroundColor Yellow
    }
}

Write-Host "`nDone!" -ForegroundColor Green
