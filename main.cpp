// Trabalho Teoria dos Grafos 01
// Luiz Guilherme Almas Araujo    
// Felipe da Costa Pinto Vilela

#include "Graph.hpp"
#include "defines.hpp"

void showMenu() {
    std::cout << "\nMenu:\n";
    std::cout << "1. Fecho transitivo direto de um vertice\n";
    std::cout << "2. Fecho transitivo indireto de um vertice\n";
    std::cout << "3. Caminho minimo entre dois vertices (Dijkstra)\n";
    std::cout << "4. Caminho minimo entre dois vertices (Floyd)\n";
    std::cout << "5. Arvore Geradora Minima (Prim)\n";
    std::cout << "6. Arvore Geradora Minima (Kruskal)\n";
    std::cout << "7. Busca em profundidade\n";
    std::cout << "8. Raio, Diametro, Centro e Periferia do grafo\n";
    std::cout << "9. Conjunto de vertices de articulacao\n";
    std::cout << "10. Salvar grafo na lista de adjacência\n";
    std::cout << "0. Sair\n";
    std::cout << "Escolha uma opcao:";
}

void sairSalvar(bool to_file, std::ofstream& output_file, const std::string& content) {
    if (to_file) {
        output_file << content;
    } else {
        std::cout << content;
    }
}

std::string converterEdge(float weight) {
    // Converte o peso da aresta para um inteiro e retorna como string
    int int_weight = static_cast<int>(weight);
    return std::to_string(int_weight);
}

int main(int argc, char* argv[]) {
    if (argc != 6) {
        std::cerr << "Uso incorreto! Uso correto: " << argv[0] << " <input_file> <output_file> <Op_Direc> <Op_PesoAresta> <Op_PesoNos>\n";
        return 1;
    }

    const char* input_file_name = argv[1];
    const char* output_file_name = argv[2];
    bool directed = std::stoi(argv[3]) == 1;
    bool weighted_edges = std::stoi(argv[4]) == 1;
    bool weighted_nodes = std::stoi(argv[5]) == 1;

    std::ifstream input_file(input_file_name);
    if (!input_file) {
        std::cerr << "Erro ao abrir o arquivo de entrada: " << input_file_name << "\n";
        return 1;
    }

    // Passa os parâmetros para o construtor de Graph
    Graph graph(input_file, directed, weighted_edges, weighted_nodes);
    input_file.close();

    std::ofstream output_file(output_file_name);
    if (!output_file) {
        std::cerr << "Erro ao abrir o arquivo de saída: " << output_file_name << "\n";
        return 1;
    }

    int option;
    do {
        showMenu();
        std::cin >> option;
        bool save_to_file = false;
        std::string result;
        
        switch(option) {
            case 1: {
                // Fecho transitivo direto de um vértice
                size_t node_id;
                std::cout << "Insira o ID do vertice: ";
                std::cin >> node_id;
                std::unordered_set<int> visitado = graph.transitivo_direto(node_id);
                result = "\nFecho transitivo direto para o vértice " + std::to_string(node_id) + ":\n";
                for (int id : visitado) {
                    result += std::to_string(id) + "\n";
                }
                break;
            }
            case 2: {
                // Fecho transitivo indireto de um vértice
                size_t node_id;
                std::cout << "Insira o ID do vertice: ";
                std::cin >> node_id;
                std::unordered_set<int> visitado = graph.transitivo_indireto(node_id);
                result = "\nFecho transitivo indireto para o vértice " + std::to_string(node_id) + ":\n";
                for (int id : visitado) {
                    result += std::to_string(id) + "\n";
                }
                break;
            }
            case 3: {
                // Caminho mínimo entre dois vértices usando Dijkstra
                size_t start_id, end_id;
                std::cout << "Insira o ID do vertice de origem: ";
                std::cin >> start_id;
                std::cout << "Insira o ID do vertice de destino: ";
                std::cin >> end_id;

                std::vector<size_t> path = graph.dijkstra(start_id, end_id);
                if (path.empty()) {
                    result = "Não há caminho entre " + std::to_string(start_id) + " e " + std::to_string(end_id) + ".\n";
                } else {
                    result = "\nCaminho mínimo de " + std::to_string(start_id) + " até " + std::to_string(end_id) + ":\n";
                    for (size_t id : path) {
                        result += std::to_string(id) + " ";
                    }
                    result += "\n";
                }
                break;
            }
            case 4: {
                // Caminho mínimo entre dois vértices usando Floyd-Warshall
                size_t start_id, end_id;
                std::cout << "Insira o ID do vertice de origem: ";
                std::cin >> start_id;
                std::cout << "Insira o ID do vertice de destino: ";
                std::cin >> end_id;

                std::vector<size_t> path = graph.floyd_warshall(start_id, end_id);
                if (path.empty()) {
                    result = "Não há caminho entre " + std::to_string(start_id) + " e " + std::to_string(end_id) + ".\n";
                } else {
                    result = "\nCaminho mínimo de " + std::to_string(start_id) + " até " + std::to_string(end_id) + ":\n";
                    for (size_t id : path) {
                        result += std::to_string(id) + " ";
                    }
                    result += "\n";
                }
                break;
            }
            case 5: {
                // Árvore Geradora Mínima usando Prim
                size_t start_id;
                std::cout << "Insira o ID do vertice inicial: ";
                std::cin >> start_id;

                Node* start_node = graph.find_node(start_id);
                if (!start_node) {
                    result = "Vértice não encontrado!\n";
                    break;
                }

                std::vector<Edge> agm_edge = graph.prim(start_id);

                // Gera a AGM no formato dot
                result = "\nGraph{\n";
                for (const Edge& edge : agm_edge) {
                    result += "  " + std::to_string(start_id) + " -- " + std::to_string(edge._target_id);
                    if (graph.sao_ponderadas()) {
                        result += " [label=\"" + converterEdge(edge._weight) + "\"]";
                    }
                    result += ";\n";
                }
                result += "}\n\n";

                break;
            }
            case 6: {
                // Árvore Geradora Mínima usando Kruskal
                size_t n;
                std::cout << "Insira o número de vértices no subconjunto: ";
                std::cin >> n;
                std::unordered_set<size_t> subconj; //armazena os IDs dos vertices que fazem parte do subconjunto do grafo
                size_t vertex_id;
                for (size_t i = 0; i < n; ++i) {
                    std::cout << "Insira o ID do vértice: ";
                    std::cin >> vertex_id;
                    subconj.insert(vertex_id);
                }

                std::vector<Edge> agm_edge = graph.kruskal_mst(subconj);

                output_file << "\nÁrvore Geradora Mínima sobre o subgrafo vértice-induzido por X:\n";
                for (const Edge& edge : agm_edge) {
                    output_file << "(" << edge._source_id << ", " << edge._target_id << ") peso: " << edge._weight << "\n";
                }
                std::cout << "\n";
                break;
            }
            case 7: {
                // Busca em profundidade
                size_t start_id;
                std::cout << "Insira o ID do vertice inicial: ";
                std::cin >> start_id;
    
                Node* start_node = graph.find_node(start_id);
                if (!start_node) {
                    result = "Vértice não encontrado!\n";
                    break;
                }

                std::unordered_set<int> visitado;
                graph.dfs_direto(start_node, visitado);

                // Adiciona uma mensagem de teste ao arquivo de saída
                result = "\nResultado do DFS começando no vértice " + std::to_string(start_id) + ":\n";
                result += "subgraph cluster_dfs {\n";
                result += "  label=\"DFS \";\n";
                for (int id : visitado) {
                    result += "  " + std::to_string(id) + ";\n";
                }
                result += "}\n";
                break;
            }
            case 8: {
                // Raio, Diâmetro, Centro e Periferia do grafo
                auto [raio, diametro, centro, periferias] = graph.calcula_raio_diametro_center_periferia();

                // Constroi a string de resultado
                result = "\nRaio do grafo: " + std::to_string(raio) + "\n";
                result += "Diâmetro do grafo: " + std::to_string(diametro) + "\n";
                result += "Centro(s) do grafo: ";
                for (size_t center : centro) {
                    result += std::to_string(center) + " ";
                }
                result += "\n";
                result += "Periferia(s) do grafo: ";
                for (size_t periferia : periferias) {
                    result += std::to_string(periferia) + " ";
                }
                result += "\n";
                break;
            }
            case 9: {
                // Conjunto de vértices de articulação
                std::unordered_set<size_t> pontos_articulacao = graph.find_pontos_articulacao();

                result = "\nConjunto de vértices de articulação:\n";
                if (pontos_articulacao.empty()) {
                    result += "Nenhum vértice de articulação encontrado.\n";
                } else {
                    for (size_t id : pontos_articulacao) {
                        result += std::to_string(id) + "\n";
                    }
                }
                break;
            }
            case 10: {
                std::string adj_list_output = "lista_adj.txt";
                graph.listaAdjacencia(adj_list_output);
                std::cout << "Lista de adjacência salva no arquivo: " << adj_list_output << "\n";
                break;
            }
            case 0:
                result = "Saindo...\n";
                break;
            default:
                result = "Opcao invalida!\n";
                break;
        }

        // Sempre imprime o resultado na tela
        std::cout << result;

        // Pergunta ao usuário se deseja salvar o resultado em arquivo, se a opção não for 0
        if (option != 0 && option != 10) {
            char save_option;
            std::cout << "\nDeseja salvar o resultado em um arquivo? (s/n): ";
            std::cin >> save_option;

            if (save_option == 's' || save_option == 'S') {
                // Salva o resultado em arquivo
                sairSalvar(true, output_file, result);
                std::cout << "Resultado salvo em " << output_file_name << "\n";
            } else {
                std::cout << "Resultado não salvo em arquivo.\n";
            }
        }

    } while (option != 0);

    output_file.close();
    return 0;
}