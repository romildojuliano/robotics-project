#!bin/bash
.DEFAULT_GOAL := help

SSHPASS_COMMAND = sshpass -p "Mec@tr0n" ssh -t robot@192.168.1.102
ARGS = $(filter-out $@,$(MAKECMDGOALS))

.PHONY: help run_command run_code

help:
	@echo "Comandos disponíveis:"
	@echo "  make run_command <comando>   - Executa um comando remoto no robô e desconecta"
	@echo "  make run_code <arquivo>      - Executa um arquivo Python remoto no robô e desconeta"

#
.PHONY: connect
connect:
	$(SSHPASS_COMMAND)

# Ex. de uso: make run_command ls
# Roda o comando ls dentro do robô e desconecta 
.PHONY: run_command
run_command:
	$(SSHPASS_COMMAND) "$(ARGS)"	

# Ex. de uso: make run_code ponteH.py
# Executa o código ponteH.py
.PHONY: run_code
run_code:
	$(SSHPASS_COMMAND) "python3 -u robot_ws/src/drivers/src/$(ARGS)"

%:
	@:
