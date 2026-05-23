@echo off
setlocal
cd /d "%~dp0"

echo Generando Repomix simulador...
echo Salida: repomix-output-simAutito.xml
echo.

call npx --yes repomix@latest --config "repomix.config.json" --no-security-check
if errorlevel 1 (
  echo.
  echo Error: no se pudo generar el Repomix completo.
  echo Verifica que Node.js/npm esten instalados y que tengas conexion para descargar Repomix la primera vez.
  pause
  exit /b 1
)

echo.
echo Listo: repomix-output-simAutito.xml
pause
