/**
 * @file struct_descriptor.hpp
 * @author RyoukiWei
 **/
#pragma once

#include <cstddef>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace rik_sd
{
/**
 * ============================
 * Layer 0: tiny type utilities
 * ============================
 * Plumbing for traits and tuple iteration.
 * Higher layers use these helpers to keep contracts local and diagnostics
 * clear. This is explicit field descriptors + generic traversal, not
 * reflection.
 */

// Contract: standard void_t alias for detection idioms.
template <class...>
using void_t = void;

// Contract: removes cv/ref qualifiers from T.
template <typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

// Contract: compatibility alias; same result as remove_cvref_t.
template <typename T>
using remove_cverf_t = remove_cvref_t<T>;

// Contract: forward declaration for metadata wrapper consumed by core traits.
template <typename Field, typename... Attr>
struct field_with_attr;

namespace detail
{
// Used by small tuple-based helpers in this header.
// Requires: Tuple is tuple-like.
template <typename Tuple, typename F, std::size_t... I>
constexpr void tuple_for_each_impl(Tuple&& tup, F&& f,
                                   std::index_sequence<I...>) {
    (f(std::get<I>(std::forward<Tuple>(tup))), ...);
}

// Used by attribute scans and other tiny tuple algorithms.
// Requires: Tuple is tuple-like.
template <typename Tuple, typename F>
constexpr void tuple_for_each(Tuple&& tup, F&& f) {
    using tuple_t = remove_cvref_t<Tuple>;
    constexpr std::size_t count = std::tuple_size<tuple_t>::value;
    tuple_for_each_impl(std::forward<Tuple>(tup), std::forward<F>(f),
                        std::make_index_sequence<count>{});
}

// Maps wrappers to their underlying field descriptor type.
// Default: core_field_t<T> is T (after cv/ref removal).
template <typename T>
struct core_field_type
{
    using type = remove_cvref_t<T>;
};

// Canonical unwrapped field type used by compile-time checks.
template <typename T>
using core_field_t = typename core_field_type<remove_cvref_t<T>>::type;

// Trait gates to avoid hard errors on tuple_size / tuple_element.
template <typename T, typename = void>
struct is_tuple_like : std::false_type
{
};

// Trait gate for field descriptor shape checks.
template <typename T, typename = void>
struct is_named_field : std::false_type
{
};

// Used by fields() contract checks.
template <typename T, std::size_t... I>
constexpr bool is_field_tuple_elements_impl(std::index_sequence<I...>) {
    using tuple_t = core_field_t<T>;
    return (is_named_field<
                remove_cvref_t<std::tuple_element_t<I, tuple_t>>>::value &&
            ...);
}

// Used by fields_of() to report descriptor shape errors.
template <typename T, bool = is_tuple_like<core_field_t<T>>::value>
struct is_field_tuple_impl : std::false_type
{
};

// Tuple-like specialization for descriptor contract checks.
template <typename T>
struct is_field_tuple_impl<T, true>
    : std::bool_constant<is_field_tuple_elements_impl<core_field_t<T>>(
          std::make_index_sequence<std::tuple_size<core_field_t<T>>::value>{})>
{
};

// True iff T is a tuple of named field descriptors (wrappers allowed).
template <typename T>
constexpr bool is_field_tuple_v = is_field_tuple_impl<T>::value;
}  // namespace detail

/**
 * ============================
 * Layer 1: field core
 * ============================
 * Minimal descriptor unit: member pointer + explicit name.
 * Layer 2 decorates this core type with metadata wrappers.
 * Layer 5 traversal only depends on this core contract (name + ptr).
 * Note: member names are never parsed; callers must provide .name(...).
 */

// Contract: immutable field descriptor with explicit name and member pointer.
template <typename Owner, typename Member>
struct basic_field
{
    using owner_type = Owner;
    using member_type = Member;

    const char* name;
    Member Owner::* ptr;

    constexpr basic_field(const char* n, Member Owner::* p) : name(n), ptr(p) {}

    // Contract: returns empty view when name is null; no allocation.
    constexpr std::string_view name_sv() const {
        return name == nullptr ? std::string_view{} : std::string_view{name};
    }
};

// Contract: intermediate builder; becomes a descriptor only after .name(...).
template <typename Owner, typename Member>
struct field_builder
{
    using owner_type = Owner;
    using member_type = Member;

    Member Owner::* ptr;

    // Contract: creates a named field descriptor from builder state.
    constexpr basic_field<Owner, Member> name_as(const char* n) const {
        return basic_field<Owner, Member>{n, ptr};
    }

    // Contract: rvalue-only API to prevent persisting/reusing unfinished
    // builders.
    constexpr basic_field<Owner, Member> name(const char* n) && {
        return name_as(n);
    }
};

// Contract: creates a builder from member pointer; caller must provide
// .name(...).
template <typename Owner, typename Member>
constexpr auto field(Member Owner::* ptr) {
    return field_builder<Owner, Member>{ptr};
}

/**
 * =====================================
 * Layer 2: optional fluent metadata
 * =====================================
 * Metadata wrapper layer over named core fields.
 * Layer 1 provides the field contract; this layer only adds attrs.
 * Layer 4/5 still validate and traverse through the unwrapped core field.
 * Note: with_attrs(field_builder, ...) is deleted to make forgot .name() fail
 * fast.
 */

// Contract: wraps a named field with arbitrary attribute tuple.
template <typename Field, typename... Attr>
struct field_with_attr
{
    Field field;
    std::tuple<Attr...> attrs;

    constexpr field_with_attr(Field f, Attr... a)
        : field(f), attrs(std::make_tuple(a...)) {}
};

namespace detail
{
// Used by fields() contract checks; avoids hard tuple_size errors.
template <typename T>
struct is_tuple_like<T,
                     void_t<decltype(std::tuple_size<core_field_t<T>>::value)>>
    : std::true_type
{
};

// Used by make_descriptor() and fields_of() contract checks.
template <typename T>
struct is_named_field<
    T, void_t<decltype(std::declval<const core_field_t<T>&>().name),
              decltype(std::declval<const core_field_t<T>&>().ptr)>>
    : std::bool_constant<
          std::is_member_pointer<
              decltype(std::declval<const core_field_t<T>&>().ptr)>::value &&
          std::is_convertible<
              decltype(std::declval<const core_field_t<T>&>().name),
              std::string_view>::value>
{
};

// Unwrap metadata wrappers for trait checks.
template <typename Field, typename... Attr>
struct core_field_type<field_with_attr<Field, Attr...>>
{
    using type = remove_cvref_t<Field>;
};
}  // namespace detail

// Contract: attaches attributes to an already named field.
template <typename Owner, typename Member, typename... Attr>
constexpr auto with_attrs(basic_field<Owner, Member> f, Attr... attrs) {
    return field_with_attr<basic_field<Owner, Member>, Attr...>{f, attrs...};
}

// Contract: intentionally deleted to force explicit naming before metadata.
template <typename Owner, typename Member, typename... Attr>
constexpr auto with_attrs(field_builder<Owner, Member>, Attr...)
    -> field_with_attr<basic_field<Owner, Member>, Attr...> = delete;

// Contract: convenience constructor for a named field descriptor.
template <typename Owner, typename Member>
constexpr auto named(Member Owner::* ptr, const char* name) {
    return field(ptr).name(name);
}

// Contract: convenience constructor for named field + attributes.
template <typename Owner, typename Member, typename... Attr,
          typename std::enable_if<(sizeof...(Attr) > 0), int>::type = 0>
constexpr auto named(Member Owner::* ptr, const char* name, Attr... attrs) {
    return with_attrs(field(ptr).name(name), attrs...);
}

// Contract: creates descriptor tuple and enforces "tuple of named fields" at
// callsite.
template <typename... FieldLike>
constexpr auto make_descriptor(FieldLike... fields) {
    static_assert(
        (detail::is_named_field<remove_cvref_t<FieldLike>>::value && ...),
        "rik_sd::make_descriptor(...): every element must be a named field. "
        "Use rik_sd::named(...) or rik_sd::field(...).name(...).");
    return std::make_tuple(fields...);
}

/**
 * ============================
 * Layer 3: descriptor<T>
 * ============================
 * User extension point for describing structs.
 * Each described type specializes descriptor<T> and defines fields().
 * Layer 4 discovers these specializations and validates the fields() contract.
 * Note: fields() must return a tuple of named field descriptors.
 */

// Contract: primary template; users specialize and provide static fields().
template <typename T>
struct descriptor;

/**
 * =================================
 * Layer 4: detection trait (SFINAE)
 * =================================
 * Compile-time contract layer for descriptor availability and shape.
 * Layer 3 provides descriptor<T>::fields(); Layer 5 APIs are gated here.
 * fields_of<T>() centralizes checks so diagnostics stay consistent.
 * Note: descriptor<T>::fields() must satisfy is_field_tuple_v.
 */

// Contract: false by default; specialized via detection when fields() exists.
template <typename, typename = void>
struct has_descriptor : std::false_type
{
};

// Contract: true when descriptor<remove_cvref_t<T>>::fields() is well-formed.
template <typename T>
struct has_descriptor<T,
                      void_t<decltype(descriptor<remove_cvref_t<T>>::fields())>>
    : std::true_type
{
};

// Contract: shorthand boolean for descriptor availability.
template <typename T>
constexpr bool has_descriptor_v = has_descriptor<T>::value;

// Contract: returns descriptor fields; triggers static_assert on
// missing/invalid descriptor shape.
template <typename T>
constexpr auto fields_of()
    -> decltype(descriptor<remove_cvref_t<T>>::fields()) {
    static_assert(
        has_descriptor_v<T>,
        "rik_sd::fields_of<T>: missing rik_sd::descriptor<T> specialization.");
    using tuple_t = decltype(descriptor<remove_cvref_t<T>>::fields());
    static_assert(detail::is_field_tuple_v<tuple_t>,
                  "rik_sd::descriptor<T>::fields() must return tuple of named "
                  "field descriptors.");
    return descriptor<remove_cvref_t<T>>::fields();
}

// Contract: alias to the compile-time field tuple type for T.
template <typename T>
using fields_t = decltype(fields_of<T>());

// Contract: compile-time field count; requires valid fields_of<T>().
template <typename T>
constexpr std::size_t field_count_v = std::tuple_size<fields_t<T>>::value;

/**
 * ============================
 * Layer 5: generic traversal
 * ============================
 * Generic traversal and query APIs over explicit descriptors.
 * Consumes Layer 4 contracts and stays agnostic to concrete user structs.
 * Runtime name lookups in this layer are linear scans over fields (O(N)).
 * Note: recursion/visits only enter explicitly described types.
 */

namespace detail
{
// Used by attrs_of() for plain fields; avoids allocating empty tuples
// repeatedly.
inline constexpr std::tuple<> empty_attrs{};

// Used by traversal/query code to handle wrapped and unwrapped fields
// uniformly.
template <typename FieldLike>
constexpr decltype(auto) unwrap_field(const FieldLike& fld) {
    return fld;
}

// Used by traversal/query code for metadata wrappers.
template <typename Field, typename... Attr>
constexpr const Field& unwrap_field(
    const field_with_attr<Field, Attr...>& fld) {
    return fld.field;
}

// Used by attribute queries on plain fields.
template <typename FieldLike>
constexpr auto attrs_of(const FieldLike&) -> const std::tuple<>& {
    return empty_attrs;
}

// Used by attribute queries on wrapped fields.
template <typename Field, typename... Attr>
constexpr auto attrs_of(const field_with_attr<Field, Attr...>& fld)
    -> const std::tuple<Attr...>& {
    return fld.attrs;
}

// Used by for_each()/visit()/find_field()/try_get() non-const paths.
template <typename T, typename F>
constexpr void for_each_impl(T& obj, F&& f) {
    auto tup = fields_of<T>();
    std::apply(
        [&](const auto&... fld_like) {
            (f(fld_like, obj.*(unwrap_field(fld_like).ptr)), ...);
        },
        tup);
}

// Used by const traversal/query paths.
template <typename T, typename F>
constexpr void for_each_impl(const T& obj, F&& f) {
    auto tup = fields_of<T>();
    std::apply(
        [&](const auto&... fld_like) {
            (f(fld_like, obj.*(unwrap_field(fld_like).ptr)), ...);
        },
        tup);
}

// Used by visit_recursive() non-const path.
// Note: only recurses into described member types.
template <typename T, typename F,
          typename std::enable_if<has_descriptor_v<T>, int>::type = 0>
constexpr void visit_recursive_impl(T& obj, F&& f) {
    for_each_impl(obj, [&](const auto& fld, auto& value) {
        f(fld, value);
        using member_t = remove_cvref_t<decltype(value)>;
        if constexpr (has_descriptor_v<member_t>) {
            visit_recursive_impl(value, f);
        }
    });
}

// Used by visit_recursive() const path.
template <typename T, typename F,
          typename std::enable_if<has_descriptor_v<T>, int>::type = 0>
constexpr void visit_recursive_impl(const T& obj, F&& f) {
    for_each_impl(obj, [&](const auto& fld, const auto& value) {
        f(fld, value);
        using member_t = remove_cvref_t<decltype(value)>;
        if constexpr (has_descriptor_v<member_t>) {
            visit_recursive_impl(value, f);
        }
    });
}
}  // namespace detail

// Contract: returns the underlying core field from either plain or wrapped
// field.
template <typename FieldLike>
constexpr decltype(auto) core_field(const FieldLike& fld) {
    return detail::unwrap_field(fld);
}

// Contract: visits each described member; enabled only for described T.
template <typename T, typename F,
          typename std::enable_if<has_descriptor_v<T>, int>::type = 0>
constexpr void for_each(T& obj, F&& f) {
    detail::for_each_impl(obj, std::forward<F>(f));
}

// Contract: const overload of for_each; callback sees const member refs.
template <typename T, typename F,
          typename std::enable_if<has_descriptor_v<T>, int>::type = 0>
constexpr void for_each(const T& obj, F&& f) {
    detail::for_each_impl(obj, std::forward<F>(f));
}

// Contract: alias API for for_each; no extra side effects.
template <typename T, typename F,
          typename std::enable_if<has_descriptor_v<T>, int>::type = 0>
constexpr void visit(T& obj, F&& f) {
    for_each(obj, std::forward<F>(f));
}

// Contract: const alias API for for_each.
template <typename T, typename F,
          typename std::enable_if<has_descriptor_v<T>, int>::type = 0>
constexpr void visit(const T& obj, F&& f) {
    for_each(obj, std::forward<F>(f));
}

// Contract: recursive traversal over described nested members only.
template <typename T, typename F,
          typename std::enable_if<has_descriptor_v<T>, int>::type = 0>
constexpr void visit_recursive(T& obj, F&& f) {
    detail::visit_recursive_impl(obj, std::forward<F>(f));
}

// Contract: const recursive traversal over described nested members.
template <typename T, typename F,
          typename std::enable_if<has_descriptor_v<T>, int>::type = 0>
constexpr void visit_recursive(const T& obj, F&& f) {
    detail::visit_recursive_impl(obj, std::forward<F>(f));
}

// Contract: runtime name lookup (O(N)); invokes callback at most once when
// matched.
template <
    typename T, typename F,
    typename std::enable_if<has_descriptor_v<remove_cvref_t<T>>, int>::type = 0>
bool find_field(T& obj, std::string_view name, F&& f) {
    bool found = false;
    for_each(obj, [&](const auto& fld, auto& member) {
        const auto& core = core_field(fld);
        if (found || core.name_sv() != name) {
            return;
        }
        found = true;
        f(fld, member);
    });
    return found;
}

// Contract: fallback for non-described types; always returns false.
template <typename T, typename F,
          typename std::enable_if<!has_descriptor_v<remove_cvref_t<T>>,
                                  int>::type = 0>
bool find_field(T&, std::string_view, F&&) {
    return false;
}

// Contract: runtime name lookup (O(N)) + exact type match before assigning out.
template <
    typename T, typename U,
    typename std::enable_if<has_descriptor_v<remove_cvref_t<T>>, int>::type = 0>
bool try_get(T& obj, std::string_view name, U& out) {
    bool matched = false;
    (void)find_field(obj, name, [&](const auto&, auto& member) {
        using member_t = remove_cvref_t<decltype(member)>;
        if constexpr (std::is_same<member_t, U>::value) {
            out = member;
            matched = true;
        }
    });
    return matched;
}

// Contract: concatenates descriptor tuples for composition (e.g., inheritance).
template <typename... Tuples>
constexpr auto concat_fields(Tuples&&... tpls) {
    return std::tuple_cat(std::forward<Tuples>(tpls)...);
}

// Contract: scans attribute tuple by type; runtime cost is O(N_attr).
template <typename Attr, typename FieldLike>
constexpr bool has_attr(const FieldLike& fld) {
    bool found = false;
    detail::tuple_for_each(detail::attrs_of(fld), [&](const auto& attr) {
        using attr_t = remove_cvref_t<decltype(attr)>;
        if constexpr (std::is_same<Attr, attr_t>::value) {
            found = true;
        }
    });
    return found;
}

// Contract: scans attribute tuple by type (O(N_attr)); invokes f for each
// matching attr.
template <typename Attr, typename FieldLike, typename F>
constexpr bool with_attr(const FieldLike& fld, F&& f) {
    bool found = false;
    detail::tuple_for_each(detail::attrs_of(fld), [&](const auto& attr) {
        using attr_t = remove_cvref_t<decltype(attr)>;
        if constexpr (std::is_same<Attr, attr_t>::value) {
            found = true;
            f(attr);
        }
    });
    return found;
}

}  // namespace rik_sd