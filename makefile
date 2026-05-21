BOBSIM_DIR := BobSim
VEHICLE_CONFIG ?= vehicles/current/vehicle.yml
TIRES_DIR ?= vehicles/current/tires
PYTHON ?= $(shell if [ -x .venv/bin/python ]; then printf ".venv/bin/python"; else printf "python3"; fi)
BOBSIM_VEHICLE := $(BOBSIM_DIR)/vehicle.yml
BOBSIM_TIRE_TEMPLATES := $(BOBSIM_DIR)/_0_Utils/external/BobLib/Generation/tire_templates

.DEFAULT_GOAL := help

.PHONY: help init setup sync-inputs sync-vehicle sync-tires study-catalog report-index study-vdyn-001 study-vdyn-002 study-vdyn-003 study-vdyn-004 study-vdyn-005 study-vdyn-006 study-vdyn-007 study-vdyn-008 study-vdyn-009 study-vdyn-010 study-vdyn-011 study-vdyn-012 study-vdyn-013 study-vdyn-014 study-vdyn-015 study-vdyn-016 study-vdyn-017 study-vdyn-018-plan study-vdyn-018-standardsim study-vdyn-all study-aero-001 study-aero-002 study-aero-003 study-aero-all study-chassis-001 study-chassis-002 study-chassis-003 study-chassis-all study-design-event-crosswalk study-design-event-requirements study-design-event-interfaces study-design-event-validation study-design-event-risk study-design-event-question-bank study-design-event-all study-all bobsim-standard-build bobsim-setup update-submodules submodule-status doctor

help:
	@printf "lhre-simulation targets:\n"
	@printf "  make init               Initialize BobSim and nested submodules\n"
	@printf "  make setup              Initialize submodules and build BobSim containers\n"
	@printf "  make sync-inputs        Stage LHRE vehicle and tire inputs into BobSim\n"
	@printf "  make sync-vehicle       Stage VEHICLE_CONFIG into BobSim/vehicle.yml\n"
	@printf "  make sync-tires         Stage TIRES_DIR .tir files into BobSim\n"
	@printf "  make study-catalog      Show the fresh purpose-first study catalog\n"
	@printf "  make report-index       Show the fresh split report index\n"
	@printf "  make study-vdyn-001     Run VDYN-001 source vehicle audit\n"
	@printf "  make study-vdyn-002     Run VDYN-002 baseline envelope\n"
	@printf "  make study-vdyn-018-plan  Generate StandardSim hardpoint calibration design\n"
	@printf "  make study-vdyn-018-standardsim  Compile/run 25 StandardSim hardpoint variants\n"
	@printf "  make study-vdyn-all     Run all vehicle dynamics studies\n"
	@printf "  make study-aero-001     Run AERO-001 map and reference audit\n"
	@printf "  make study-aero-all     Run all aero studies\n"
	@printf "  make study-chassis-all  Run all chassis studies\n"
	@printf "  make study-design-event-crosswalk  Run Design Event rubric crosswalk\n"
	@printf "  make study-design-event-all  Run all Design Event integration studies\n"
	@printf "  make study-all          Run all current purpose-defined studies\n"
	@printf "  make bobsim-standard-build  Build BobSim StandardSim executables\n"
	@printf "  make bobsim-setup       Run BobSim's setup target\n"
	@printf "  make update-submodules  Pull submodules to their configured remote heads\n"
	@printf "  make submodule-status   Show pinned submodule commits\n"
	@printf "  make doctor             Check required local tools\n"

init:
	git submodule update --init --recursive

setup: init bobsim-setup
	@printf "lhre-simulation setup complete.\n"

sync-inputs: sync-vehicle sync-tires
	@printf "LHRE simulation inputs are staged in BobSim.\n"

sync-vehicle:
	@test -f "$(VEHICLE_CONFIG)" || { printf "Missing vehicle config: $(VEHICLE_CONFIG)\n"; exit 1; }
	cp "$(VEHICLE_CONFIG)" "$(BOBSIM_VEHICLE)"
	@printf "Synced %s -> %s\n" "$(VEHICLE_CONFIG)" "$(BOBSIM_VEHICLE)"

sync-tires:
	@test -d "$(TIRES_DIR)" || { printf "Missing tire data directory: $(TIRES_DIR)\n"; exit 1; }
	@mkdir -p "$(BOBSIM_TIRE_TEMPLATES)"
	cp "$(TIRES_DIR)"/*.tir "$(BOBSIM_TIRE_TEMPLATES)/"
	@printf "Synced tire templates from %s -> %s\n" "$(TIRES_DIR)" "$(BOBSIM_TIRE_TEMPLATES)"

study-catalog:
	@sed -n '1,220p' studies/README.md

bobsim-standard-build: sync-inputs
	$(MAKE) -C $(BOBSIM_DIR) sync-vehicle-yaml build-records build-vehicle-sim build-four-post-sim PYTHON=$(abspath $(PYTHON))

report-index:
	@sed -n '1,220p' reports/2026-design-report-index.md

study-vdyn-001:
	$(PYTHON) studies/vdyn/VDYN-001-source-vehicle-audit/run.py

study-vdyn-002: study-vdyn-001
	$(PYTHON) studies/vdyn/VDYN-002-baseline-envelope/run.py

study-vdyn-003:
	$(PYTHON) studies/vdyn/VDYN-003-standardsim-baseline/run.py

study-vdyn-004: study-vdyn-002
	$(PYTHON) studies/vdyn/VDYN-004-tire-operating-window/run.py

study-vdyn-005: study-vdyn-003
	$(PYTHON) studies/vdyn/VDYN-005-setup-authority/run.py

study-vdyn-006: study-vdyn-002
	$(PYTHON) studies/vdyn/VDYN-006-tire-load-sensitivity/run.py

study-vdyn-007:
	$(PYTHON) studies/vdyn/VDYN-007-tire-pure-slip-curves/run.py

study-vdyn-008: study-vdyn-002 study-vdyn-003
	$(PYTHON) studies/vdyn/VDYN-008-tire-cornering-stiffness/run.py

study-vdyn-009: study-vdyn-003
	$(PYTHON) studies/vdyn/VDYN-009-tire-relaxation-response/run.py

study-vdyn-010: study-vdyn-002
	$(PYTHON) studies/vdyn/VDYN-010-tire-combined-slip-budget/run.py

study-vdyn-011: study-vdyn-002
	$(PYTHON) studies/vdyn/VDYN-011-envelope-doe-importance/run.py

study-vdyn-012: study-vdyn-002
	$(PYTHON) studies/vdyn/VDYN-012-aero-scaling-doe/run.py

study-vdyn-013: study-vdyn-003
	$(PYTHON) studies/vdyn/VDYN-013-torsional-stiffness-authority/run.py

study-vdyn-014:
	$(PYTHON) studies/vdyn/VDYN-014-static-alignment-screening/run.py

study-vdyn-015: study-vdyn-002
	$(PYTHON) studies/vdyn/VDYN-015-envelopesim-interaction-doe/run.py

study-vdyn-016: study-vdyn-003 study-vdyn-008 study-vdyn-009 study-vdyn-013 study-vdyn-014
	$(PYTHON) studies/vdyn/VDYN-016-standardsim-response-surface-doe/run.py

study-vdyn-017: study-vdyn-003
	$(PYTHON) studies/vdyn/VDYN-017-hardpoint-monte-carlo-tolerance/run.py

study-vdyn-018-plan: study-vdyn-003 study-vdyn-017
	$(PYTHON) studies/vdyn/VDYN-018-standardsim-hardpoint-calibration/run.py

study-vdyn-018-standardsim: study-vdyn-003 study-vdyn-017
	$(PYTHON) studies/vdyn/VDYN-018-standardsim-hardpoint-calibration/run.py --run-standardsim --cases 25 --max-workers 4

study-vdyn-all: study-vdyn-001 study-vdyn-002 study-vdyn-003 study-vdyn-004 study-vdyn-005 study-vdyn-006 study-vdyn-007 study-vdyn-008 study-vdyn-009 study-vdyn-010 study-vdyn-011 study-vdyn-012 study-vdyn-013 study-vdyn-014 study-vdyn-015 study-vdyn-016 study-vdyn-017 study-vdyn-018-plan

study-aero-001:
	$(PYTHON) studies/aero/AERO-001-map-and-reference-audit/run.py

study-aero-002: study-aero-001
	$(PYTHON) studies/aero/AERO-002-platform-sensitivity/run.py

study-aero-003: study-aero-001 study-vdyn-002
	$(PYTHON) studies/aero/AERO-003-vehicle-integration/run.py

study-aero-all: study-aero-001 study-aero-002 study-aero-003

study-chassis-001: study-vdyn-001
	$(PYTHON) studies/chassis/CHASSIS-001-source-and-hardpoint-audit/run.py

study-chassis-002: study-vdyn-002 study-aero-001
	$(PYTHON) studies/chassis/CHASSIS-002-load-case-generation/run.py

study-chassis-003: study-vdyn-003
	$(PYTHON) studies/chassis/CHASSIS-003-stiffness-and-validation/run.py

study-chassis-all: study-chassis-001 study-chassis-002 study-chassis-003

study-design-event-crosswalk: study-vdyn-all study-aero-all study-chassis-all
	$(PYTHON) studies/design-event/DE-001-rubric-crosswalk/run.py

study-design-event-requirements: study-design-event-crosswalk
	$(PYTHON) studies/design-event/DE-002-requirements-traceability/run.py

study-design-event-interfaces: study-design-event-requirements
	$(PYTHON) studies/design-event/DE-003-interface-control-matrix/run.py

study-design-event-validation: study-design-event-interfaces
	$(PYTHON) studies/design-event/DE-004-validation-correlation-plan/run.py

study-design-event-risk: study-design-event-validation
	$(PYTHON) studies/design-event/DE-005-risk-correlation-priority/run.py

study-design-event-question-bank: study-design-event-risk
	$(PYTHON) studies/design-event/DE-006-judge-question-bank/run.py

study-design-event-all: study-design-event-crosswalk study-design-event-requirements study-design-event-interfaces study-design-event-validation study-design-event-risk study-design-event-question-bank

study-all: study-vdyn-all study-aero-all study-chassis-all study-design-event-all

bobsim-setup:
	$(MAKE) -C $(BOBSIM_DIR) setup

update-submodules:
	git submodule update --remote --recursive

submodule-status:
	git submodule status --recursive

doctor:
	@command -v git >/dev/null || { printf "Missing required tool: git\n"; exit 1; }
	@command -v docker >/dev/null || { printf "Missing required tool: docker\n"; exit 1; }
	@docker compose version >/dev/null || { printf "Docker Compose is not available.\n"; exit 1; }
	@test -d "$(BOBSIM_DIR)" || { printf "Missing BobSim submodule directory.\n"; exit 1; }
	@test -f "$(VEHICLE_CONFIG)" || { printf "Missing vehicle config: $(VEHICLE_CONFIG)\n"; exit 1; }
	@test -d "$(TIRES_DIR)" || { printf "Missing tire data directory: $(TIRES_DIR)\n"; exit 1; }
	@printf "Required setup tools and vehicle inputs are available.\n"
