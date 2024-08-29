#include "Graph.hpp"
#include "Conjunto.hpp"
#include "Node.hpp"

Graph::Graph(std::ifstream& instance, bool directed, bool weighted_edges, bool weighted_nodes)
    : _number_of_nodes(0), _number_of_edges(0), _directed(directed),
      _weighted_edges(weighted_edges), _weighted_nodes(weighted_nodes),
      _first(nullptr), _last(nullptr)
{
    std::string line;
    size_t node_id_1, node_id_2;
    float weight;

    // Lê a primeira linha para obter o número de vértices
    if (std::getline(instance, line)) {
        std::istringstream iss(line);
        size_t num_vertices;
        if (iss >> num_vertices) {
            // Cria os vértices
            for (size_t i = 1; i <= num_vertices; ++i) {
                add_node(i);  // Adiciona os nós com ID de 1 até num_vertices
                std::cout << "Nó adicionado: " << i << " com peso: 0\n";
            }
        } else {
            std::cerr << "Número de vértices inválido na primeira linha do arquivo.\n";
            return;
        }
    }

    // Processa as arestas
    while (std::getline(instance, line)) {
        std::istringstream iss(line);

        // Verifica se a linha tem o formato esperado
        if (!(iss >> node_id_1 >> node_id_2 >> weight)) {
            std::cerr << "Formato inválido na linha: " << line << std::endl;
            continue; // Pula para a próxima linha
        }

        // Mensagem de depuração para verificar a leitura dos valores
        std::cout << "Leitura bruta: " << node_id_1 << " " << node_id_2 << " " << weight << "\n";

        // Adiciona aresta
        add_edge(node_id_1, node_id_2, weight);

        // Se o grafo não for direcionado, adicione a aresta na direção oposta também
        if (!_directed) {
            add_edge(node_id_2, node_id_1, weight);
        }

        // Mensagens de depuração
        std::cout << "Aresta adicionada: " << node_id_1 << " -> " << node_id_2 << " com peso: " << weight << "\n";
    }
}



Graph::Graph()
    : _number_of_nodes(0), _number_of_edges(0), _directed(false),
      _weighted_edges(false), _weighted_nodes(false), _first(nullptr), _last(nullptr)
{
}

Graph::~Graph()
{
    Node* atual = _first;
    while (atual) {
        Node* next = atual->_next_node;
        Edge* edge = atual->_first_edge;
        while (edge) {
            Edge* next_edge = edge->_next_edge;
            delete edge;
            edge = next_edge;
        }
        delete atual;
        atual = next;
    }
}

void Graph::remove_node(size_t node_id)
{
    Node* node_to_remove = find_node(node_id);
    if (!node_to_remove) return;

    Edge* edge = node_to_remove->_first_edge;
    while (edge) {
        Edge* next_edge = edge->_next_edge;
        if (!_directed) {
            Node* target_node = find_node(edge->_target_id);
            if (target_node) {
                Edge* target_edge = target_node->_first_edge;
                Edge* prev_edge = nullptr;
                while (target_edge) {
                    if (target_edge->_target_id == node_id) {
                        if (prev_edge) prev_edge->_next_edge = target_edge->_next_edge;
                        else target_node->_first_edge = target_edge->_next_edge;
                        delete target_edge;
                        break;
                    }
                    prev_edge = target_edge;
                    target_edge = target_edge->_next_edge;
                }
            }
        }
        delete edge;
        edge = next_edge;
    }

    if (node_to_remove == _first) _first = node_to_remove->_next_node;
    if (node_to_remove == _last) _last = node_to_remove->_previous_node;
    if (node_to_remove->_previous_node) node_to_remove->_previous_node->_next_node = node_to_remove->_next_node;
    if (node_to_remove->_next_node) node_to_remove->_next_node->_previous_node = node_to_remove->_previous_node;

    delete node_to_remove;
    --_number_of_nodes;
}

void Graph::remove_edge(size_t node_id_1, size_t node_id_2)
{
    Node* node1 = find_node(node_id_1);
    Node* node2 = find_node(node_id_2);
    if (!node1 || !node2) return;

    Edge* edge = node1->_first_edge;
    Edge* prev_edge = nullptr;
    while (edge) {
        if (edge->_target_id == node_id_2) {
            if (prev_edge) prev_edge->_next_edge = edge->_next_edge;
            else node1->_first_edge = edge->_next_edge;
            delete edge;
            break;
        }
        prev_edge = edge;
        edge = edge->_next_edge;
    }

    if (!_directed) {
        edge = node2->_first_edge;
        prev_edge = nullptr;
        while (edge) {
            if (edge->_target_id == node_id_1) {
                if (prev_edge) prev_edge->_next_edge = edge->_next_edge;
                else node2->_first_edge = edge->_next_edge;
                delete edge;
                break;
            }
            prev_edge = edge;
            edge = edge->_next_edge;
        }
    }
    --_number_of_edges;
}

void Graph::add_node(size_t node_id, float weight)
{
    // Verifica se o nó já existe
    if (find_node(node_id)) {
        std::cout << "Nó " << node_id << " já existe. Não adicionado.\n"; // Depuração
        return;
    }

    // Cria um novo nó
    Node* new_node = new Node;
    new_node->_id = node_id;
    new_node->_weight = weight;
    new_node->_number_of_edges = 0;
    new_node->_first_edge = nullptr;
    new_node->_next_node = nullptr;
    new_node->_previous_node = _last;

    if (_last) _last->_next_node = new_node;
    else _first = new_node;

    _last = new_node;
    ++_number_of_nodes;

}


void Graph::add_edge(size_t node_id_1, size_t node_id_2, float weight)
{
    Node* node1 = find_node(node_id_1);
    Node* node2 = find_node(node_id_2);

    if (!node1 || !node2) {
        std::cout << "Não é possível adicionar aresta. Um ou ambos os nós não existem.\n"; // Depuração
        return;
    }

    // Adiciona aresta de node_id_1 para node_id_2
    Edge* new_edge = new Edge;
    new_edge->_target_id = node_id_2;
    new_edge->_weight = weight;
    new_edge->_next_edge = node1->_first_edge;
    node1->_first_edge = new_edge;
    ++node1->_number_of_edges;

    // Se o grafo não for direcionado, adicione a aresta na direção oposta
    if (!_directed) {
        new_edge = new Edge;
        new_edge->_target_id = node_id_1;
        new_edge->_weight = weight;
        new_edge->_next_edge = node2->_first_edge;
        node2->_first_edge = new_edge;
        ++node2->_number_of_edges;
    }
    ++_number_of_edges;
}


void Graph::print_graph(std::ofstream& output_file)
{
    output_file << "digraph G {\n";
    Node* atual = _first;
    while (atual) {
        Edge* edge = atual->_first_edge;
        while (edge) {
            output_file << "  " << atual->_id << " -> " << edge->_target_id;
            if (_weighted_edges) {
                output_file << " [label=\"" << edge->_weight << "\"]";
            }
            output_file << ";\n";
            edge = edge->_next_edge;
        }
        atual = atual->_next_node;
    }
    output_file << "}\n";
}

int Graph::connected(size_t node_id_1, size_t node_id_2)
{
    Node* node1 = find_node(node_id_1);
    Node* node2 = find_node(node_id_2);
    if (!node1 || !node2) return 0;

    Edge* edge = node1->_first_edge;
    while (edge) {
        if (edge->_target_id == node_id_2) {
            return 1; // Conectado
        }
        edge = edge->_next_edge;
    }
    return 0; // Não conectado
}

Node* Graph::find_node(size_t node_id)
{
    Node* atual = _first;
    while (atual) {
        if (atual->_id == node_id) {
            return atual;
        }
        atual = atual->_next_node;
    }
    return nullptr;
}

std::unordered_set<int> Graph::transitivo_direto(size_t vertex_id)
{
    std::unordered_set<int> direct_vizinhos;
    Node* start_node = find_node(vertex_id);
    if (!start_node) return direct_vizinhos;

    // Adiciona apenas os vértices diretamente conectados
    Edge* edge = start_node->_first_edge;
    while (edge) {
        direct_vizinhos.insert(edge->_target_id);
        edge = edge->_next_edge;
    }

    return direct_vizinhos;
}


std::unordered_set<int> Graph::transitivo_indireto(size_t vertex_id)
{
    std::unordered_set<int> visitado;
    Node* start_node = find_node(vertex_id);
    if (!start_node) return visitado;

    // Inicia a DFS a partir do nó inicial
    dfs_indireto(start_node, visitado);

    // Remove o vértice de origem do conjunto de visitados, se estiver presente
    visitado.erase(vertex_id);

    return visitado;
}

void Graph::dfs_direto(Node* node, std::unordered_set<int>& visitado)
{
    if (visitado.find(node->_id) != visitado.end()) return;

    visitado.insert(node->_id);
    Edge* edge = node->_first_edge;
    while (edge) {
        Node* vizinho = find_node(edge->_target_id);
        if (vizinho) dfs_direto(vizinho, visitado);
        edge = edge->_next_edge;
    }
}

void Graph::dfs_indireto(Node* node, std::unordered_set<int>& visitado)
{
    if (!node || visitado.count(node->_id)) return;

    visitado.insert(node->_id);

    for (Edge* edge = node->_first_edge; edge; edge = edge->_next_edge) {
        Node* vizinho = find_node(edge->_target_id);
        if (vizinho) {
            dfs_indireto(vizinho, visitado);
        }
    }
}

std::vector<size_t> Graph::dijkstra(size_t start_id, size_t end_id) {
    std::unordered_map<size_t, float> distancias;
    std::unordered_map<size_t, size_t> anteriores;
    for (Node* node = _first; node != nullptr; node = node->_next_node) {
        distancias[node->_id] = std::numeric_limits<float>::infinity();
        anteriores[node->_id] = -1; // Indica que não tem anteriores
    }
    distancias[start_id] = 0;

    auto compare = [&distancias](size_t esq, size_t dir) {
        return distancias[esq] > distancias[dir];
    };
    std::priority_queue<size_t, std::vector<size_t>, decltype(compare)> pq(compare);
    pq.push(start_id);

    while (!pq.empty()) {
        size_t atual = pq.top();
        pq.pop();

        Node* node_atual = find_node(atual);
        if (!node_atual) continue;

        for (Edge* edge = node_atual->_first_edge; edge != nullptr; edge = edge->_next_edge) {
            size_t vizinho_id = edge->_target_id;
            float weight = edge->_weight;
            float nova_dist = distancias[atual] + weight;

            if (nova_dist < distancias[vizinho_id]) {
                distancias[vizinho_id] = nova_dist;
                anteriores[vizinho_id] = atual;
                pq.push(vizinho_id);
            }
        }
    }

    std::vector<size_t> path;
    for (size_t at = end_id; at != -1; at = anteriores[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    if (path.front() != start_id) {
        return {}; // Caminho não encontrado
    }

    return path;
}

std::vector<size_t> Graph::floyd_warshall(size_t start_id, size_t end_id) {
    std::unordered_map<size_t, std::unordered_map<size_t, float>> dist;
    std::unordered_map<size_t, std::unordered_map<size_t, size_t>> next;

    // Inicializa as distâncias e os anteriores
    for (Node* u = _first; u != nullptr; u = u->_next_node) {
        size_t u_id = u->_id;
        dist[u_id][u_id] = 0;  // Distancia de um nó pra ele mesmo é 0
        next[u_id][u_id] = u_id;
        
        for (Edge* e = u->_first_edge; e != nullptr; e = e->_next_edge) {
            dist[u_id][e->_target_id] = e->_weight;
            next[u_id][e->_target_id] = e->_target_id;
        }
    }

    // Algoritmo de Floyd
    for (Node* k = _first; k != nullptr; k = k->_next_node) {
        size_t k_id = k->_id;
        for (Node* i = _first; i != nullptr; i = i->_next_node) {
            size_t i_id = i->_id;
            for (Node* j = _first; j != nullptr; j = j->_next_node) {
                size_t j_id = j->_id;
                if (dist[i_id].count(k_id) && dist[k_id].count(j_id)) {
                    float nova_dist = dist[i_id][k_id] + dist[k_id][j_id];
                    auto it = dist[i_id].find(j_id);
                    if (it == dist[i_id].end() || nova_dist < it->second) {
                        dist[i_id][j_id] = nova_dist;
                        next[i_id][j_id] = next[i_id][k_id];
                    }
                }
            }
        }
    }

    // Reconstrói o caminho
    std::vector<size_t> path;
    size_t u = start_id;
    path.push_back(u);

    while (u != end_id) {
        if (next[u].count(end_id) == 0) {
            return {};  // Caminho não encontrado
        }
        u = next[u][end_id];
        path.push_back(u);
    }

    return path;
}

     // Árvore Geradora Mínima usando Prim
std::vector<Edge> Graph::prim(size_t start_id) {
    std::vector<Edge> agm_edges;
    std::unordered_set<size_t> in_agm;
    auto compare = [](Edge* e1, Edge* e2) { return e1->_weight > e2->_weight; };
    std::priority_queue<Edge*, std::vector<Edge*>, decltype(compare)> pq(compare);

    Node* start_node = find_node(start_id);
    if (!start_node) return agm_edges;

    // Adiciona as arestas do nó inicial à fila de prioridades
    Edge* edge = start_node->_first_edge;
    while (edge) {
        pq.push(edge);
        edge = edge->_next_edge;
    }
    in_agm.insert(start_node->_id);

    while (!pq.empty()) {
        Edge* min_edge = pq.top();
        pq.pop();

        if (in_agm.find(min_edge->_target_id) == in_agm.end()) {
            agm_edges.push_back(*min_edge);

            Node* next_node = find_node(min_edge->_target_id);
            if (next_node) {
                in_agm.insert(next_node->_id);
                edge = next_node->_first_edge;
                while (edge) {
                    if (in_agm.find(edge->_target_id) == in_agm.end()) {
                        pq.push(edge);
                    }
                    edge = edge->_next_edge;
                }
            }
        }
    }

    return agm_edges;
}

bool Graph::sao_ponderadas() const {
    return _weighted_edges;
}


// Implementação do método kruskal
std::vector<Edge> Graph::kruskal_mst(std::unordered_set<size_t> subset) {
    std::vector<Edge> agm_edges;
    std::vector<Edge> edges;

    // Coleta as arestas que conectam os vértices do subconjunto
    for (Node* node = _first; node != nullptr; node = node->_next_node) {
        if (subset.find(node->_id) != subset.end()) {
            Edge* edge = node->_first_edge;
            while (edge) {
                if (subset.find(edge->_target_id) != subset.end()) {
                    edges.push_back(*edge);
                }
                edge = edge->_next_edge;
            }
        }
    }

    // Ordena as arestas por peso
    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
        return a._weight < b._weight;
    });

    // Inicializa o Disjoint Set
    conjunto ds(_number_of_nodes);

    // Adiciona arestas ao MST usando o algoritmo de Kruskal
    for (const Edge& edge : edges) {
        size_t root1 = ds.find(edge._source_id);
        size_t root2 = ds.find(edge._target_id);

        if (root1 != root2) {
            agm_edges.push_back(edge);
            ds.union_sets(root1, root2);
        }
    }

    return agm_edges;
}


//Raio, Diâmetro, Centro e Periferia do grafo
std::tuple<float, float, std::unordered_set<size_t>, std::unordered_set<size_t>> Graph::calcula_raio_diametro_center_periferia() {
    // Calcula as distâncias mínimas entre todos os pares de vértices usando Floyd
    std::unordered_map<size_t, std::unordered_map<size_t, float>> dist;
    std::unordered_map<size_t, std::unordered_map<size_t, size_t>> next;

    for (Node* u = _first; u != nullptr; u = u->_next_node) {
        size_t u_id = u->_id;
        dist[u_id][u_id] = 0;
        next[u_id][u_id] = u_id;
        
        for (Edge* e = u->_first_edge; e != nullptr; e = e->_next_edge) {
            dist[u_id][e->_target_id] = e->_weight;
            next[u_id][e->_target_id] = e->_target_id;
        }
    }

    for (Node* k = _first; k != nullptr; k = k->_next_node) {
        size_t k_id = k->_id;
        for (Node* i = _first; i != nullptr; i = i->_next_node) {
            size_t i_id = i->_id;
            for (Node* j = _first; j != nullptr; j = j->_next_node) {
                size_t j_id = j->_id;
                if (dist[i_id].count(k_id) && dist[k_id].count(j_id)) {
                    float nova_dist = dist[i_id][k_id] + dist[k_id][j_id];
                    if (dist[i_id].find(j_id) == dist[i_id].end() || nova_dist < dist[i_id][j_id]) {
                        dist[i_id][j_id] = nova_dist;
                        next[i_id][j_id] = next[i_id][k_id];
                    }
                }
            }
        }
    }

    // Calcular o raio, diâmetro, centro e periferia
    float raio = std::numeric_limits<float>::infinity();
    float diametro = 0;
    std::unordered_map<size_t, float> excentricidades;
    std::unordered_set<size_t> centros;
    std::unordered_set<size_t> periferias;

    for (Node* u = _first; u != nullptr; u = u->_next_node) {
        size_t u_id = u->_id;
        float excentricidade = 0;
        for (Node* v = _first; v != nullptr; v = v->_next_node) {
            size_t v_id = v->_id;
            if (dist[u_id].count(v_id)) {
                excentricidade = std::max(excentricidade, dist[u_id][v_id]);
            }
        }
        excentricidades[u_id] = excentricidade;
        raio = std::min(raio, excentricidade);
        diametro = std::max(diametro, excentricidade);
    }

    for (const auto& [node_id, ecc] : excentricidades) {
        if (ecc == raio) {
            centros.insert(node_id);
        }
        if (ecc == diametro) {
            periferias.insert(node_id);
        }
    }

    // Retorna os resultados
    return std::make_tuple(raio, diametro, centros, periferias);
}

//Conjunto de vertices de articulacao
void Graph::dfs_articulacao(
    size_t node_id,
    size_t& tempo,
    std::unordered_map<size_t, size_t>& descobre_tempo,
    std::unordered_map<size_t, size_t>& menor_tempo,
    std::unordered_map<size_t, size_t>& pai,
    std::unordered_set<size_t>& pontos_articulacao,
    std::unordered_set<size_t>& visitado
) {
    descobre_tempo[node_id] = menor_tempo[node_id] = ++tempo;
    visitado.insert(node_id);
    size_t filho = 0;

    Node* node = find_node(node_id);
    if (!node) return;

    for (Edge* edge = node->_first_edge; edge; edge = edge->_next_edge) {
        size_t vizinho_id = edge->_target_id;

        if (descobre_tempo.find(vizinho_id) == descobre_tempo.end()) {
            // Se o vizinho não for visitado
            pai[vizinho_id] = node_id;
            ++filho;

            dfs_articulacao(vizinho_id, tempo, descobre_tempo, menor_tempo, pai, pontos_articulacao, visitado);

            menor_tempo[node_id] = std::min(menor_tempo[node_id], menor_tempo[vizinho_id]);

            if (pai[node_id] == 0 && filho > 1) {
                pontos_articulacao.insert(node_id);
            } else if (pai[node_id] != 0 && menor_tempo[vizinho_id] >= descobre_tempo[node_id]) {
                pontos_articulacao.insert(node_id);
            }
        } else if (vizinho_id != pai[node_id]) {
            // Atualizar valor baixo do nó
            menor_tempo[node_id] = std::min(menor_tempo[node_id], descobre_tempo[vizinho_id]);
        }
    }
}

std::unordered_set<size_t> Graph::find_pontos_articulacao() {
    std::unordered_set<size_t> pontos_articulacao;
    std::unordered_map<size_t, size_t> descobre_tempo;
    std::unordered_map<size_t, size_t> menor_tempo;
    std::unordered_map<size_t, size_t> pai;
    std::unordered_set<size_t> visitado;
    size_t tempo = 0;

    Node* node = _first;
    while (node) {
        if (descobre_tempo.find(node->_id) == descobre_tempo.end()) {
            dfs_articulacao(node->_id, tempo, descobre_tempo, menor_tempo, pai, pontos_articulacao, visitado);
        }
        node = node->_next_node;
    }

    return pontos_articulacao;
}


// Salva o grafo em lista de Adjacencia
void Graph::listaAdjacencia(const std::string& filename) const {
    std::ofstream output_file(filename);
    if (!output_file) {
        std::cerr << "Erro ao abrir o arquivo de saída: " << filename << "\n";
        return;
    }

    // Mapa para armazenar a lista de adjacência sem duplicações
    std::map<size_t, std::set<std::pair<size_t, float>>> lista_adj;

    Node* node_atual = _first;
    while (node_atual != nullptr) {
        Edge* edge_atual = node_atual->_first_edge;
        while (edge_atual != nullptr) {
            // Adiciona a aresta de node_atual para edge_atual->_target_id
            lista_adj[node_atual->_id].emplace(edge_atual->_target_id, edge_atual->_weight);

            // Se o grafo não for direcionado, verifica a aresta reversa e a remove
            if (!_directed) {
                auto& target_set = lista_adj[edge_atual->_target_id];
                auto edge_reversa = std::make_pair(node_atual->_id, edge_atual->_weight);
                if (target_set.find(edge_reversa) != target_set.end()) {
                    target_set.erase(edge_reversa);
                }
            }

            edge_atual = edge_atual->_next_edge;
        }
        node_atual = node_atual->_next_node;
    }

    // Escreve a lista de adjacência no arquivo
    for (const auto& [node_id, edges] : lista_adj) {
        output_file << node_id << ": ";
        bool first_edge = true;
        for (const auto& [target_id, peso] : edges) {
            if (!first_edge) {
                output_file << " ";
            }
            output_file << target_id;
            if (sao_ponderadas()) {
                output_file << " (" << static_cast<int>(peso) << ")";
            }
            first_edge = false;
        }
        output_file << "\n";
    }

}



//void Graph::set_directed(bool directed) { _directed = directed; }
//void Graph::set_weighted_edges(bool weighted_edges) { _weighted_edges = weighted_edges; }
//void Graph::set_weighted_nodes(bool weighted_nodes) { _weighted_nodes = weighted_nodes; }