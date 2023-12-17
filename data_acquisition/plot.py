import csv
import matplotlib.pyplot as plt

def plotar_diagrama_retangulos():
    # Lista para armazenar os dados
    data = []

    with open('dados.csv', newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            task_id = float(row['TaskId'])
            start_time = float(row['StartTime(s)'])
            end_time = float(row['EndTime(s)'])
            duration = end_time - start_time

            data.append({'Task': task_id, 'Start': start_time, 'End': end_time, 'Duration': duration})

    # Criar um gráfico de retângulos
    fig, ax = plt.subplots()

    # Definir esquema de cores
    colors = {0: 'red', 1: 'green', 2: 'blue'}

    for task in data:
        task_id = task['Task']

        if task_id not in colors:
            continue  # Pular tarefas com TaskId não mapeado para cor

        rect = plt.Rectangle((task['Start'], task_id - 0.2), task['Duration'], 0.4, color=colors[task_id], label=f'Task {task_id}')
        ax.add_patch(rect)

    # Adicionar título e rótulos aos eixos
    ax.set_title('Diagrama de Execução de Tarefas')
    ax.set_xlabel('Tempo (s)')
    ax.set_ylabel('Tarefa')

    # Exibir o gráfico
    plt.yticks([0, 1, 2])  # Defina os ticks para corresponder às tarefas
    # plt.legend()
    plt.grid(True)

    # Salvar o gráfico em um arquivo
    plt.savefig('diagrama_tarefas.png')

    # Exibir o gráfico
    plt.show()

if __name__ == "__main__":
    plotar_diagrama_retangulos()
