import serial
import csv

ser = serial.Serial('/dev/ttyUSB0', 9600)

def receber_dados():
    with open('dados.csv', 'w', newline='') as csvfile:
        campos = ['TaskId', 'StartTime(s)', 'EndTime(s)']
        writer = csv.DictWriter(csvfile, fieldnames=campos)
        writer.writeheader()

        while True:
            try:
                linha = ser.readline().decode('utf-8').strip()
                print(f"linha recebida: {linha}")

                # Separa os campos usando ',' como delimitador
                valores = linha.split(',')

                # Verifica se a linha contém todos os valores esperados
                if len(valores) == 3:
                    dados = {campo.split(':')[0].strip(): float(campo.split(':')[1].strip()) for campo in valores}
                    print(f"dados: {dados}")

                    # Escreve os dados no arquivo CSV
                    writer.writerow(dados)
                else:
                    print("Erro: Esperava 3 valores, mas recebeu", len(valores))
            except UnicodeDecodeError as e:
                print(f"Erro de decodificação: {e}")
            except ValueError as e:
                print(f"Erro ao converter para número: {e}")

if __name__ == "__main__":
    receber_dados()
