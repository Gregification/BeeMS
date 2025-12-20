

Add-Type -Name Window -Namespace Console -MemberDefinition '
[DllImport("Kernel32.dll")]
public static extern IntPtr GetConsoleWindow();

[DllImport("user32.dll")]
public static extern bool ShowWindow(IntPtr hWnd, Int32 nCmdShow);
'

$console = [Console.Window]::GetConsoleWindow()

# 0 hide
[Console.Window]::ShowWindow($console, 0) | Out-Null




Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

function Show-Calculator {
    $form = New-Object Windows.Forms.Form
    $form.Text = "KiCad Price Calculator (v1.2.1)"
    $form.Size = New-Object Drawing.Size(750, 850)
    $form.MinimumSize = New-Object Drawing.Size(600, 700)
    $form.StartPosition = "CenterScreen"

    # --- INPUT SECTION ---
    $label = New-Object Windows.Forms.Label
    $label.Text = "Input (Paste KiCad Schematic Data):"
    $label.Location = New-Object Drawing.Point(10, 10)
    $label.Size = New-Object Drawing.Size(300, 20)
    $form.Controls.Add($label)

    $inputBox = New-Object Windows.Forms.RichTextBox
    $inputBox.Location = New-Object Drawing.Point(10, 35)
    $inputBox.Size = New-Object Drawing.Size(710, 200)
    $inputBox.ScrollBars = "Both"
    $inputBox.WordWrap = $false
    $inputBox.Anchor = "Top, Left, Right"
    $form.Controls.Add($inputBox)

    # --- BUTTONS ---
    $btnPaste = New-Object Windows.Forms.Button
    $btnPaste.Text = "1. Paste Clipboard"
    $btnPaste.Location = New-Object Drawing.Point(10, 245)
    $btnPaste.Size = New-Object Drawing.Size(130, 45)
    $btnPaste.Add_Click({ $inputBox.Text = [Windows.Forms.Clipboard]::GetText() })
    $form.Controls.Add($btnPaste)

    $btnCalc = New-Object Windows.Forms.Button
    $btnCalc.Text = "2. Calculate All"
    $btnCalc.Location = New-Object Drawing.Point(145, 245)
    $btnCalc.Size = New-Object Drawing.Size(130, 45)
    $form.Controls.Add($btnCalc)

    # --- DASHBOARD PANEL (Stats) ---
    $statsGroup = New-Object Windows.Forms.GroupBox
    $statsGroup.Text = "Dashboard"
    $statsGroup.Location = New-Object Drawing.Point(285, 238)
    $statsGroup.Size = New-Object Drawing.Size(435, 55)
    $statsGroup.Anchor = "Top, Left, Right"
    $form.Controls.Add($statsGroup)

    $lblTotal = New-Object Windows.Forms.Label
    $lblTotal.Text = "Total: $0.00"
    $lblTotal.Location = New-Object Drawing.Point(10, 22)
    $lblTotal.Size = New-Object Drawing.Size(140, 20)
    $lblTotal.Font = New-Object Drawing.Font("Segoe UI", 10, [Drawing.FontStyle]::Bold)
    $statsGroup.Controls.Add($lblTotal)

    $lblWarn = New-Object Windows.Forms.Label
    $lblWarn.Text = "Tier Warnings: 0"
    $lblWarn.Location = New-Object Drawing.Point(150, 22)
    $lblWarn.Size = New-Object Drawing.Size(130, 20)
    $statsGroup.Controls.Add($lblWarn)

    $lblFail = New-Object Windows.Forms.Label
    $lblFail.Text = "Missing Prices: 0"
    $lblFail.Location = New-Object Drawing.Point(290, 22)
    $lblFail.Size = New-Object Drawing.Size(130, 20)
    $statsGroup.Controls.Add($lblFail)

    # --- RESULT SECTION ---
    $resultDisplay = New-Object Windows.Forms.RichTextBox
    $resultDisplay.ReadOnly = $true
    $resultDisplay.Font = New-Object Drawing.Font("Consolas", 10)
    $resultDisplay.Location = New-Object Drawing.Point(10, 305)
    $resultDisplay.Size = New-Object Drawing.Size(710, 480)
    $resultDisplay.BackColor = "White"
    $resultDisplay.ScrollBars = "Both" 
    $resultDisplay.Anchor = "Top, Bottom, Left, Right"
    $form.Controls.Add($resultDisplay)

    # --- LOGIC ---
    $btnCalc.Add_Click({
        $resultDisplay.Clear()
        $rawText = $inputBox.Text
        if ([string]::IsNullOrWhiteSpace($rawText)) { return }

        $partsTable = @{}
        $blocks = [regex]::Split($rawText, '\(symbol(?=\s+\(lib_id)') | Where-Object { $_ -match '\(property "mouser"' }

        foreach ($block in $blocks) {
            $refMatch = [regex]::Match($block, '\(property "Reference" "(?<ref>[^"]+)"')
            $mouserMatch = [regex]::Match($block, '\(property "mouser" "(?<link>[^"]+)"')
            
            if ($refMatch.Success -and $mouserMatch.Success) {
                $ref = $refMatch.Groups['ref'].Value
                $link = $mouserMatch.Groups['link'].Value

                if (-not $partsTable.ContainsKey($link)) {
                    $partsTable[$link] = @{ Tiers = @{}; Refs = New-Object System.Collections.Generic.List[string] }
                }
                if (-not $partsTable[$link].Refs.Contains($ref)) { $partsTable[$link].Refs.Add($ref) }

                $tierMatches = [regex]::Matches($block, '\(property "unit x(?<qty>\d+)"\s+"(?<price>[\d\.]+)"')
                foreach ($m in $tierMatches) {
                    $partsTable[$link].Tiers[[int]$m.Groups['qty'].Value] = [double]$m.Groups['price'].Value
                }
            }
        }

        $grandTotal = 0.0
        $warningCount = 0
        $failCount = 0

        $resultDisplay.AppendText("PARTS BREAKDOWN:`n" + ("-" * 60) + "`n")

        foreach ($link in $partsTable.Keys) {
            $part = $partsTable[$link]
            $totalQty = $part.Refs.Count
            $tierQtys = $part.Tiers.Keys | Sort-Object

            $unitPrice = 0.0
            $isFallback = $false
            $noPrice = $false

            if ($tierQtys.Count -gt 0) {
                $bestTierQty = $tierQtys | Where-Object { $_ -le $totalQty } | Select-Object -Last 1
                if ($null -ne $bestTierQty) {
                    $unitPrice = $part.Tiers[$bestTierQty]
                } else {
                    $unitPrice = $part.Tiers[$tierQtys[0]]
                    $isFallback = $true
                    $warningCount++
                }
            } else {
                $noPrice = $true
                $failCount++
            }

            $subTotal = $unitPrice * $totalQty
            $grandTotal += $subTotal

            # Color coding
            if ($noPrice) { $resultDisplay.SelectionColor = [Drawing.Color]::Red }
            elseif ($isFallback) { $resultDisplay.SelectionColor = [Drawing.Color]::Goldenrod }
            else { $resultDisplay.SelectionColor = [Drawing.Color]::DarkBlue }
            
            $resultDisplay.AppendText("Part: $link`n")
            $resultDisplay.SelectionColor = [Drawing.Color]::Black
            $resultDisplay.AppendText("Refs: " + (($part.Refs | Sort-Object) -join ", ") + "`n")
            
            $priceText = "Qty: $totalQty @ $" + $unitPrice.ToString("F3") + " each"
            if ($noPrice) { $priceText += " [MISSING PRICE DATA]" }
            elseif ($isFallback) { $priceText += " (Used x" + $tierQtys[0] + " tier)" }
            
            $resultDisplay.AppendText($priceText + "`n")
            $resultDisplay.AppendText("Subtotal: $" + $subTotal.ToString("F2") + "`n")
            $resultDisplay.AppendText("-" * 40 + "`n")
        }

        # Update Dashboard
        $lblTotal.Text = "Total: $" + $grandTotal.ToString("F2")
        $lblTotal.ForeColor = if ($grandTotal -gt 0) { [Drawing.Color]::DarkGreen } else { [Drawing.Color]::Black }
        $lblWarn.Text = "Tier Warnings: $warningCount"
        $lblWarn.ForeColor = if ($warningCount -gt 0) { [Drawing.Color]::Goldenrod } else { [Drawing.Color]::Gray }
        $lblFail.Text = "Missing Prices: $failCount"
        $lblFail.ForeColor = if ($failCount -gt 0) { [Drawing.Color]::Red } else { [Drawing.Color]::Gray }

        $resultDisplay.AppendText("`r`nCalculation Complete.")
    })

    $form.ShowDialog() | Out-Null
}

Show-Calculator