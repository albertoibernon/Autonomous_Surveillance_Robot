import xml.etree.ElementTree as ET
import pandas as pd
import matplotlib.pyplot as plt
import math
def parse_xml(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Buscar en la sección 'vertex'
    vertex_values = []
    position_values = [] 
    orientation_values = []
    landmarks_values = []
    for face in root.findall('.//FaceSetPart'):
        for vertex in face.findall('.//vertex'):
            vertex_text = vertex.text.strip().replace('{', '').replace('}', '')
            valores = [list(map(float, linea.split(','))) for linea in vertex_text.strip().split('\n')]
            coord=valores[2][1]
            vertex_values.append(coord)
        # Buscar en la sección 'position
        for position in face.findall('.//position'):
            position_text = position.text.strip().replace('{', '').replace('}', '')
            valores = [list(map(float, linea.split(','))) for linea in position_text.strip().split('\n')]
            coord=(valores[0][0],valores[0][1])
            position_values.append(valores)
            
        for position in face.findall('.//orientation'):
            position_text = position.text.strip().replace('{', '').replace('}', '')
            valores = [list(map(float, linea.split(','))) for linea in position_text.strip().split('\n')]
            ori=valores[0][2]
            orientation_values.append(ori)
    for land in root.findall('.//LandMark'):
        # Buscar en la sección 'position
        for position in land.findall('.//position'):
            position_text = position.text.strip().replace('{', '').replace('}', '')
            valores = [list(map(float, linea.split(','))) for linea in position_text.strip().split('\n')]
            coord=(valores[0][0],valores[0][1])
            landmarks_values.append(valores)

    coords=[]
    coords_landmarks=[]
    # Crear una figura y ejes
    fig, ax = plt.subplots()
    
    for i in range(len(landmarks_values)):
        x1=landmarks_values[i][0][0]
        y1=landmarks_values[i][0][1]
        coords_landmarks.append([x1,y1])
        ax.scatter(x1, y1, color='green', label='Balizas')
    
    
    for i in range(len(vertex_values)):
        x1=0
        y1=0
        x2=0
        y2=0
        if(orientation_values[i] == 0): #vertical
            x1=position_values[i][0][0]
            y1=position_values[i][0][1]
            y2=vertex_values[i]+position_values[i][0][1]
            x2=position_values[i][0][0]
        elif (orientation_values[i] == -1.57): #horizontal
            x1=position_values[i][0][0]
            y1=position_values[i][0][1]
            x2=vertex_values[i]+position_values[i][0][0]
            y2=position_values[i][0][1]
        else:
            x1=position_values[i][0][0]
            y1=position_values[i][0][1]
            x2=40
            y2=25
        coords.append([x1,y1,x2,y2])   
        # Mostrar la gráfica
        # Graficar los dos puntos
        # ax.scatter([x1, x2], [y1, y2], color='red', label='Puntos')
        # Dibujar la recta entre los dos puntos
        ax.plot([x1, x2], [y1, y2], color='blue', linestyle='-', linewidth=2, label='Recta')
    plt.show()
    return coords, coords_landmarks
        # values = [float(val) for val in position_text.split(',') if val.strip()]
        # position_values.append(values)

    # return vertex_values, position_values

def main():
    file_path = 'EntornoPrado.xml'  # Reemplaza con la ruta de tu archivo XML
    coords,landmarks_values=parse_xml(file_path)
    # # Crear un DataFrame de pandas para las paredes
    # df = pd.DataFrame(coords, columns=['X1', 'Y1','X2', 'Y2'])
    # df = df.iloc[1:]
    
    # # Especificar la ruta del archivo Excel
    # archivo_excel = 'mapa.xlsx'
    
    # # Guardar el DataFrame en un archivo de Excel
    # df.to_excel(archivo_excel, index=False)
    

    # # Crear un DataFrame de pandas para balizas
    # df = pd.DataFrame(landmarks_values, columns=['X1', 'Y1'])
    # # Especificar la ruta del archivo Excel
    # archivo_excel = 'mapa_balizas.xlsx'
    
    # # Guardar el DataFrame en un archivo de Excel
    # df.to_excel(archivo_excel, index=False)
    
    # print(f"Archivo Excel '{archivo_excel}' generado exitosamente.")
    

if __name__ == "__main__":
    main()