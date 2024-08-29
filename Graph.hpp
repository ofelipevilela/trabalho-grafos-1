#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "Node.hpp"
#include "defines.hpp"
#include "Conjunto.hpp"

class Graph
{
public:
    Graph(std::ifstream& instance, bool directed, bool weighted_edges, bool weighted_nodes);    
    Graph();
    ~Graph();

    void remove_node(size_t node_id);
    void remove_edge(size_t node_id_1, size_t node_id_2);
    void add_node(size_t node_id, float weight = 0);
    void add_edge(size_t node_id_1, size_t node_id_2, float weight = 0);
    void print_graph(std::ofstream& output_file);
    void print_graph();
    int connected(size_t node_id_1, size_t node_id_2);

    // Métodos adicionados
    std::unordered_set<int> transitivo_direto(size_t vertex_id);
    std::unordered_set<int> transitivo_indireto(size_t vertex_id);
    void dfs_direto(Node* node, std::unordered_set<int>& visited);

    Node* find_node(size_t node_id);
    
    // Adicionando a função dijkstra e floyd
    std::vector<size_t> dijkstra(size_t start_id, size_t end_id);
    std::vector<size_t> floyd_warshall(size_t start_id, size_t end_id);

    // Prim
    bool sao_ponderadas() const; // Declaração correta do método
    std::vector<Edge> prim(size_t start_id); // Declaração do método prim_mst

    // Kruskal
    std::vector<Edge> kruskal_mst(std::unordered_set<size_t> subset);

    // Raio, Diâmetro, Centro e Periferia do grafo
    std::tuple<float, float, std::unordered_set<size_t>, std::unordered_set<size_t>> calcula_raio_diametro_center_periferia();

    // Conjunto de vértices de articulação
    std::unordered_set<size_t> find_pontos_articulacao();

    void listaAdjacencia(const std::string& filename) const;

private:
    size_t _number_of_nodes;
    size_t _number_of_edges;
    bool   _directed;
    bool   _weighted_edges;
    bool   _weighted_nodes;
    Node  *_first;
    Node  *_last;

    void dfs_indireto(Node* node, std::unordered_set<int>& visited);

    // Conjunto de vértices de articulação
    void dfs_articulacao(
        size_t node_id,
        size_t& time,
        std::unordered_map<size_t, size_t>& descobre_tempo,
        std::unordered_map<size_t, size_t>& menor_tempo,
        std::unordered_map<size_t, size_t>& pai,
        std::unordered_set<size_t>& pontos_articulacao,
        std::unordered_set<size_t>& visitado
    );
    
};


#endif  // GRAPH_HPP