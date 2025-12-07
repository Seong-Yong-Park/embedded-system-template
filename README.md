# Embedded System Project Template

A standard project structure template for embedded system development.

## Quick Start

1. Copy this template to your project directory
2. Update this README with your project-specific information
3. Fill in the documentation templates in `docs/`
4. Start developing in `firmware/`, `hardware/`, etc.

## Project Structure

```
your-project/
├─ README.md          # Project overview, build/run instructions
├─ docs/              # Documentation (SRS, SAD, SW-ARCH, TestPlan, etc.)
├─ firmware/          # Embedded SW (app, drivers, bsp, os, config, tests)
├─ hardware/          # Circuit/PCB (schematics, pcb)
├─ mech/              # Mechanical/CAD files (optional)
├─ tools/             # Development utilities (pc-tools, scripts)
├─ ai/                # AI prompts, rulesets, workflows
└─ .github/workflows/ # CI/CD automation
```

## Key Documents

- [Project Vision](docs/00_project-vision.md)
- [Requirements Specification](docs/10_SRS.md)
- [System Architecture](docs/20_SAD.md)
- [Software Architecture](docs/21_SW-ARCH.md)
- [Test Plan](docs/30_TestPlan.md)
- [Interface Specification](docs/40_InterfaceSpec.md)
- [Changelog](docs/90_Changelog.md)

## Build and Run

Update this section with your project-specific build instructions.

```bash
# Example
cd firmware
make
make flash
```

## License

[License Information]
